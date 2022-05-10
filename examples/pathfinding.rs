use byteorder::{LittleEndian, ReadBytesExt};
use log::{info, trace, LevelFilter};

use divert::{
    DivertResult, DtStraightPathFlags, NavMesh, NavMeshParams, NavMeshQuery, PolyRef, QueryFilter,
    Vector,
};

use std::{
    cmp,
    collections::HashMap,
    error::Error,
    fs::File,
    io::{self, Read},
};

#[repr(C)]
#[derive(Debug)]
pub struct MmapTileHeader {
    magic: u32,
    dt_version: u32,
    mmap_version: u32,
    size: u32,
    use_liquids: u32,
}

impl MmapTileHeader {
    fn from_reader(mut rdr: impl Read) -> io::Result<Self> {
        let magic = rdr.read_u32::<LittleEndian>()?;
        let dt_version = rdr.read_u32::<LittleEndian>()?;
        let mmap_version = rdr.read_u32::<LittleEndian>()?;

        let size = rdr.read_u32::<LittleEndian>()?;
        let use_liquids = rdr.read_u32::<LittleEndian>()?;

        Ok(MmapTileHeader {
            magic,
            dt_version,
            mmap_version,
            size,
            use_liquids,
        })
    }
}

fn in_range(source: &Vector, destination: &Vector, radius: f32, height: f32) -> bool {
    let dx = destination.y - source.y;
    let dy = destination.z - source.z;
    let dz = destination.x - source.x;
    (dx * dx + dz * dz) < radius * radius && dy.abs() < height
}

fn world_to_trinity(world_x: f32, world_y: f32) -> (u32, u32) {
    (
        (32.0 - (world_x / 533.3333)) as u32,
        (32.0 - (world_y / 533.3333)) as u32,
    )
}

fn read_nav_mesh_params_from(mut rdr: impl Read) -> io::Result<NavMeshParams> {
    let origin_x = rdr.read_f32::<LittleEndian>()?;
    let origin_y = rdr.read_f32::<LittleEndian>()?;
    let origin_z = rdr.read_f32::<LittleEndian>()?;

    let tile_width = rdr.read_f32::<LittleEndian>()?;
    let tile_height = rdr.read_f32::<LittleEndian>()?;

    let max_tiles = rdr.read_i32::<LittleEndian>()?;
    let max_polys = rdr.read_i32::<LittleEndian>()?;

    Ok(NavMeshParams {
        origin: [origin_x, origin_y, origin_z],
        tile_width,
        tile_height,
        max_tiles,
        max_polys,
    })
}

fn find_common(this: &[PolyRef], other: &[PolyRef]) -> Option<(usize, usize)> {
    for i in (0..this.len()).rev() {
        for j in (0..other.len()).rev() {
            if this[i] == other[j] {
                return Some((i, j));
            }
        }
    }
    None
}

fn fix_up_corridor(path: &mut Vec<PolyRef>, visited: &[PolyRef]) {
    let (furthest_path, furthest_visited) = find_common(path, visited).unwrap();

    let required = visited.len() - furthest_visited;
    let orig = cmp::min(furthest_path + 1, path.len());
    let mut size = cmp::max(0, path.len() - orig);

    if required + size > path.capacity() {
        size = path.capacity() - required;
    }

    unsafe {
        path.set_len(required + size);
    };

    if size > 0 {
        unsafe {
            std::ptr::copy(
                path.as_mut_ptr().offset(orig.try_into().unwrap()),
                path.as_mut_ptr().offset(required.try_into().unwrap()),
                size,
            );
        }
    }

    for i in 0..required {
        path[i] = visited[(visited.len() - 1) - i];
    }
}

trait TileProvider {
    fn read_tile_data(&self, tile_x: u32, tile_y: u32) -> io::Result<Vec<u8>>;
}

struct TrinityTileProvider {}

impl TileProvider for TrinityTileProvider {
    fn read_tile_data(&self, tile_x: u32, tile_y: u32) -> io::Result<Vec<u8>> {
        let mut tile_file = File::open(
            &[
                "resources/geometry/530",
                &tile_x.to_string(),
                &tile_y.to_string(),
                ".mmtile",
            ]
            .join(""),
        )?;

        /*
        TrinityCore Tiles have some header information unrelated to RecastNavigation
        Here we parse it out, and just denote the header is an unused variable. Would also be valid to simply skip the 20 bytes
        tile_file.seek(SeekFrom::Current(20))?;
        */
        let _trinity_header = MmapTileHeader::from_reader(&tile_file)?;

        let mut tile_data = Vec::new();
        tile_file.read_to_end(&mut tile_data)?;
        Ok(tile_data)
    }
}

struct NavigatorSettings {
    pub max_path: i32,
    pub max_smooth_path: i32,
    pub max_steer_points: i32,
    pub steer_target_radius: f32,
    pub steer_target_height: f32,
    pub max_move_visits: i32,
    pub smooth_step_size: f32,
}

impl Default for NavigatorSettings {
    fn default() -> Self {
        Self {
            max_path: 64,
            max_smooth_path: 256,
            max_steer_points: 3,
            steer_target_radius: 0.3,
            steer_target_height: 1000.0,
            max_move_visits: 16,
            smooth_step_size: 2.0,
        }
    }
}

struct Navigator<'a> {
    tile_provider: Box<dyn TileProvider>,
    nav_mesh: NavMesh<'a>,
    nav_mesh_query: NavMeshQuery<'a>,
    query_filter: QueryFilter<'a>,
    tile_map: HashMap<u32, bool>,
    settings: NavigatorSettings,
}

impl<'a> Navigator<'a> {
    fn new(
        nav_mesh: NavMesh<'a>,
        nav_mesh_query: NavMeshQuery<'a>,
        query_filter: QueryFilter<'a>,
    ) -> Self {
        Self {
            tile_provider: Box::new(TrinityTileProvider {}),
            nav_mesh,
            nav_mesh_query,
            query_filter,
            tile_map: HashMap::with_capacity(8),
            settings: NavigatorSettings::default(),
        }
    }

    fn find_nearest_poly(&mut self, position: &Vector) -> DivertResult<(PolyRef, Vector)> {
        let extents = Vector::from_yzx(3.0f32, 5.0f32, 3.0f32);
        self.nav_mesh_query
            .find_nearest_poly(position, &extents, &self.query_filter)
    }

    fn packed_tile_id(tile_x: u32, tile_y: u32) -> u32 {
        (tile_x << 16 | tile_y) as u32
    }

    fn has_tile(&self, tile_x: u32, tile_y: u32) -> bool {
        self.tile_map
            .contains_key(&Self::packed_tile_id(tile_x, tile_y))
    }

    fn add_tile(&mut self, tile_x: u32, tile_y: u32) -> DivertResult<()> {
        info!("[FindPath] Loading Tile ({}, {})", tile_x, tile_y);

        self.nav_mesh
            .add_tile(self.tile_provider.read_tile_data(tile_x, tile_y).unwrap())?;

        self.tile_map
            .insert(Self::packed_tile_id(tile_x, tile_y), true);

        Ok(())
    }

    fn get_steer_target(
        &mut self,
        start_pos: &Vector,
        end_pos: &Vector,
        poly_path: &[PolyRef],
    ) -> DivertResult<Option<(Vector, DtStraightPathFlags, PolyRef)>> {
        let steer_points = self.nav_mesh_query.find_straight_path(
            start_pos,
            end_pos,
            poly_path,
            self.settings.max_steer_points,
            0,
        )?;

        for (mut steer_point, steer_flag, steer_poly) in steer_points {
            if steer_flag.contains(DtStraightPathFlags::OFFMESH_CONNECTION)
                || !in_range(
                    &steer_point,
                    start_pos,
                    self.settings.steer_target_radius,
                    self.settings.steer_target_height,
                )
            {
                steer_point.z = start_pos.z;
                return Ok(Some((steer_point, steer_flag, steer_poly)));
            }
        }

        Ok(None)
    }

    fn find_smooth_path(
        &mut self,
        start_pos: &Vector,
        end_pos: &Vector,
        mut polygons: Vec<PolyRef>,
    ) -> DivertResult<Vec<Vector>> {
        let mut smooth_path = Vec::with_capacity(self.settings.max_smooth_path.try_into().unwrap());

        let mut iter_pos = self
            .nav_mesh_query
            .closest_point_on_poly_boundary(*polygons.first().unwrap(), start_pos)?;

        let target_pos = self
            .nav_mesh_query
            .closest_point_on_poly_boundary(*polygons.last().unwrap(), end_pos)?;

        smooth_path.push(iter_pos);
        while !polygons.is_empty() && (smooth_path.len() < smooth_path.capacity()) {
            if let Some((steer_pos, steer_flags, _)) =
                self.get_steer_target(&iter_pos, &target_pos, &polygons)?
            {
                let delta = steer_pos - iter_pos;
                let mut len = delta.dot(&delta).sqrt();

                if steer_flags
                    .contains(DtStraightPathFlags::END | DtStraightPathFlags::OFFMESH_CONNECTION)
                    && len < self.settings.smooth_step_size
                {
                    len = 1.0;
                } else {
                    len = self.settings.smooth_step_size / len;
                }

                let move_target = iter_pos + (delta * len);
                let (move_result, visited) = self.nav_mesh_query.move_along_surface(
                    polygons[0],
                    &iter_pos,
                    &move_target,
                    &self.query_filter,
                    self.settings.max_move_visits,
                )?;

                fix_up_corridor(&mut polygons, &visited);

                let height = self
                    .nav_mesh_query
                    .get_poly_height(polygons[0], &move_result)
                    .unwrap_or(0.0);

                iter_pos = Vector::from_yzx(move_result.y, height + 0.5, move_result.x);

                smooth_path.push(iter_pos);
            } else {
                break;
            }
        }

        Ok(smooth_path)
    }

    pub fn find_path(
        &mut self,
        input_start: &Vector,
        input_end: &Vector,
    ) -> DivertResult<Vec<Vector>> {
        let start_tile = world_to_trinity(input_start.x, input_start.y);
        let end_tile = world_to_trinity(input_end.x, input_end.y);

        if !self.has_tile(start_tile.0, start_tile.1) {
            self.add_tile(start_tile.0, start_tile.1)?;
        } else {
            trace!("[FindPath] Skipping Loading of Start Tile {:?}", start_tile);
        }

        if !self.has_tile(end_tile.0, end_tile.1) {
            self.add_tile(end_tile.0, end_tile.1)?;
        } else {
            trace!("[FindPath] Skipping Loading of End Tile {:?}", end_tile);
        }

        let (start_poly, start_pos) = self.find_nearest_poly(input_start)?;
        let (end_poly, end_pos) = self.find_nearest_poly(input_end)?;

        let poly_path = self.nav_mesh_query.find_path(
            start_poly,
            end_poly,
            &start_pos,
            &end_pos,
            &self.query_filter,
            self.settings.max_path,
        )?;

        let smooth_path = self.find_smooth_path(&start_pos, &end_pos, poly_path)?;
        Ok(smooth_path)
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    pretty_env_logger::formatted_builder()
        .filter_level(LevelFilter::Info)
        .init();

    let map_params_file = File::open("resources/geometry/530.mmap")?;
    let params = read_nav_mesh_params_from(map_params_file)?;

    let nav_mesh = NavMesh::new(&params)?;
    let nav_mesh_query = NavMeshQuery::new(&nav_mesh, 2048)?;

    let mut query_filter = QueryFilter::new()?;
    query_filter.set_include_flags(1 | 8 | 4 | 2);
    query_filter.set_exclude_flags(0);

    let mut navigator = Navigator::new(nav_mesh, nav_mesh_query, query_filter);

    // // Shat Bridge (35,22) -> (35, 22)
    // let start_position = Vector::from_xyz(-1910.12, 5289.2, 1.424);
    // let end_position = Vector::from_xyz(-1931.90, 5099.05, 8.05);
    // navigator.find_path(&start_position, &end_position)?;
    // // Shat Bridge

    // CROSS_TILE
    // Terrokar (35,22) -> (35, 23)
    // let start_position = Vector::from_xyz(-1916.64, 4893.65, 2.26);
    // let end_position = Vector::from_xyz(-1947.43, 4687.55, -2.09);
    // navigator.find_path(&start_position, &end_position)?;
    // Terrokar

    // LONG Path
    // Terrokar (35,22) -> (35, 23)
    let start_position = Vector::from_xyz(-2051.9, 4350.97, 2.25);
    let end_position = Vector::from_xyz(-1916.12, 4894.67, 2.21);
    navigator
        .find_path(&start_position, &end_position)?
        .iter()
        .for_each(|position| info!("{:?}", position));
    // Terrokar

    Ok(())
}
