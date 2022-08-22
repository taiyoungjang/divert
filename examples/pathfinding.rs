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
    io::{self, Read, Seek, SeekFrom},
};

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

struct TrinityTileProvider {
    map_id: u32,
}

impl TileProvider for TrinityTileProvider {
    fn read_tile_data(&self, tile_x: u32, tile_y: u32) -> io::Result<Vec<u8>> {
        let mut tile_file = File::open(
            &[
                &format!("resources/geometry/{:03}", self.map_id),
                &tile_x.to_string(),
                &tile_y.to_string(),
                ".mmtile",
            ]
            .join(""),
        )?;

        tile_file.seek(SeekFrom::Current(20))?;
        let mut tile_data = Vec::new();
        tile_file.read_to_end(&mut tile_data)?;
        Ok(tile_data)
    }
}

struct NavigatorSettings {
    pub max_path: usize,
    pub max_smooth_path: usize,
    pub max_move_visits: usize,
    pub max_steer_points: i32,

    pub steer_target_radius: f32,
    pub steer_target_height: f32,
    pub smooth_step_size: f32,
}

impl Default for NavigatorSettings {
    fn default() -> Self {
        Self {
            max_path: 64,
            max_smooth_path: 128,
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

    poly_path: Vec<PolyRef>,
    smooth_path: Vec<Vector>,
    move_along_path: Vec<PolyRef>,
}

impl<'a> Navigator<'a> {
    fn new(map_id: u32, settings: NavigatorSettings) -> Result<Self, Box<dyn Error>> {
        let map_params_file = File::open(format!("resources/geometry/{:03}.mmap", map_id))?;
        let params = read_nav_mesh_params_from(map_params_file)?;

        let nav_mesh = NavMesh::new(&params)?;
        let nav_mesh_query = NavMeshQuery::new(&nav_mesh, 2048)?;

        let mut query_filter = QueryFilter::new()?;
        query_filter.set_include_flags(1 | 8 | 4 | 2);
        query_filter.set_exclude_flags(0);

        let max_path = settings.max_path;
        let max_smooth_path = settings.max_smooth_path;
        let max_move_along_path = settings.max_move_visits;

        Ok(Self {
            tile_provider: Box::new(TrinityTileProvider { map_id }),
            nav_mesh,
            nav_mesh_query,
            query_filter,
            tile_map: HashMap::with_capacity(8),
            settings,
            poly_path: Vec::with_capacity(max_path),
            smooth_path: Vec::with_capacity(max_smooth_path),
            move_along_path: Vec::with_capacity(max_move_along_path),
        })
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
    ) -> DivertResult<Option<(Vector, DtStraightPathFlags, PolyRef)>> {
        let steer_points = self.nav_mesh_query.find_straight_path(
            start_pos,
            end_pos,
            &self.poly_path,
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

    fn find_smooth_path(&mut self, start_pos: &Vector, end_pos: &Vector) -> DivertResult<()> {
        self.smooth_path.clear();

        let mut iter_pos = self
            .nav_mesh_query
            .closest_point_on_poly_boundary(*self.poly_path.first().unwrap(), start_pos)?;

        let target_pos = self
            .nav_mesh_query
            .closest_point_on_poly_boundary(*self.poly_path.last().unwrap(), end_pos)?;

        self.smooth_path.push(iter_pos);
        while !self.poly_path.is_empty() && (self.smooth_path.len() < self.smooth_path.capacity()) {
            if let Some((steer_pos, steer_flags, _)) =
                self.get_steer_target(&iter_pos, &target_pos)?
            {
                let delta = steer_pos - iter_pos;
                let mut len = delta.dot(&delta).sqrt();

                let end_of_path = steer_flags.contains(DtStraightPathFlags::END);
                let off_mesh_connection =
                    steer_flags.contains(DtStraightPathFlags::OFFMESH_CONNECTION);

                if (end_of_path || off_mesh_connection) && len < self.settings.smooth_step_size {
                    len = 1.0;
                } else {
                    len = self.settings.smooth_step_size / len;
                }

                let move_target = iter_pos + (delta * len);

                let mut move_result = Vector::default();
                self.nav_mesh_query.move_along_surface_inplace(
                    self.poly_path[0],
                    &iter_pos,
                    &move_target,
                    &self.query_filter,
                    &mut move_result,
                    &mut self.move_along_path,
                )?;

                fix_up_corridor(&mut self.poly_path, &self.move_along_path);

                let height = self
                    .nav_mesh_query
                    .get_poly_height(self.poly_path[0], &move_result)
                    .unwrap_or(0.0);

                iter_pos = Vector::from_yzx(move_result.y, height + 0.5, move_result.x);
                self.smooth_path.push(iter_pos);
            } else {
                break;
            }
        }

        Ok(())
    }

    pub fn find_path(&mut self, input_start: &Vector, input_end: &Vector) -> DivertResult<()> {
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

        let _poly_path_status = self.nav_mesh_query.find_path_inplace(
            start_poly,
            end_poly,
            &start_pos,
            &end_pos,
            &self.query_filter,
            &mut self.poly_path,
        )?;

        self.find_smooth_path(&start_pos, &end_pos)?;
        Ok(())
    }
}

fn test_path(
    start_position: &Vector,
    end_position: &Vector,
    navigator: &mut Navigator,
) -> Result<(), Box<dyn Error>> {
    info!(
        "Finding Path From {:?} to {:?}",
        start_position, end_position
    );

    navigator.find_path(start_position, end_position)?;

    navigator
        .smooth_path
        .iter()
        .for_each(|position| info!("{:?}", position));

    info!("Found Path from {:?} to {:?}", start_position, end_position);
    info!("Path Length: {}", navigator.smooth_path.len());

    Ok(())
}

fn main() -> Result<(), Box<dyn Error>> {
    pretty_env_logger::formatted_builder()
        .filter_level(LevelFilter::Info)
        .init();

    let mut navigator = Navigator::new(530, NavigatorSettings::default())?;

    // // Shat Bridge (35,22) -> (35, 22)
    let start_position = Vector::from_xyz(-1910.12, 5289.2, 1.424);
    let end_position = Vector::from_xyz(-1931.90, 5099.05, 8.05);
    test_path(&start_position, &end_position, &mut navigator)?;
    // // Shat Bridge

    // CROSS_TILE
    // Terrokar (35,22) -> (35, 23)
    let start_position = Vector::from_xyz(-1916.64, 4893.65, 2.26);
    let end_position = Vector::from_xyz(-1947.43, 4687.55, -2.09);
    test_path(&start_position, &end_position, &mut navigator)?;
    // Terrokar

    // LONG Path
    // Terrokar (35,22) -> (35, 23)
    let start_position = Vector::from_xyz(-2051.9, 4350.97, 2.25);
    let end_position = Vector::from_xyz(-1916.12, 4894.67, 2.21);
    test_path(&start_position, &end_position, &mut navigator)?;
    // Terrokar

    Ok(())
}
