use bitflags::bitflags;

#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct DtVector {
    pub y: f32,
    pub z: f32,
    pub x: f32,
}

pub enum DtNavMesh {}

pub enum DtNavMeshQuery {}

pub enum DtQueryFilter {}

pub type DtTileRef = u64;

pub type DtPolyRef = u64;

#[repr(C)]
#[derive(Debug)]
pub struct DtNavMeshParams {
    pub origin: [f32; 3],
    pub tile_width: f32,
    pub tile_height: f32,
    pub max_tiles: i32,
    pub max_polys: i32,
}

#[repr(C)]
#[derive(Debug)]
pub struct DtMeshHeader {
    magic: i32,
    version: i32,
    x: i32,
    y: i32,
    layer: i32,
    user_id: u32,
    poly_count: i32,
    vert_count: i32,
    max_link_count: i32,
    detail_mesh_count: i32,
    detail_vert_count: i32,
    detail_tri_count: i32,
    bv_node_count: i32,
    off_mesh_con_count: i32,
    off_mesh_base: i32,
    walkable_height: f32,
    walkable_climb: f32,
    b_min: [f32; 3],
    b_max: [f32; 3],
    bv_quant_factor: f32,
}

// High level status.
pub const DT_FAILURE: u32 = 1 << 31; // Operation failed.
pub const DT_SUCCESS: u32 = 1 << 30; // Operation succeed.
pub const DT_IN_PROGRESS: u32 = 1 << 29; // Operation still in progress.

// Detail information for status.
// pub const DT_STATUS_DETAIL_MASK: u32 = 0x0ffffff;
pub const DT_WRONG_MAGIC: u32 = 1 << 0; // Input data is not recognized.
pub const DT_WRONG_VERSION: u32 = 1 << 1; // Input data is in wrong version.
pub const DT_OUT_OF_MEMORY: u32 = 1 << 2; // Operation ran out of memory.
pub const DT_INVALID_PARAM: u32 = 1 << 3; // An input parameter was invalid.
pub const DT_BUFFER_TOO_SMALL: u32 = 1 << 4; // Result buffer for the query was too small to store all results.
pub const DT_OUT_OF_NODES: u32 = 1 << 5; // Query ran out of nodes during search.
pub const DT_PARTIAL_RESULT: u32 = 1 << 6; // Query did not reach the end location, returning best guess.
pub const DT_ALREADY_OCCUPIED: u32 = 1 << 7; // A tile has already been assigned to the given x,y coordinate

bitflags! {
    #[repr(C)]
    pub struct DtStatus: u32 {
        const SUCCESS = DT_SUCCESS;
        const IN_PROGRESS = DT_IN_PROGRESS;
        const FAILURE = DT_FAILURE;

        const WRONG_MAGIC = DT_WRONG_MAGIC;
        const WRONG_VERSION = DT_WRONG_VERSION;
        const OUT_OF_MEMORY = DT_OUT_OF_MEMORY;
        const INVALID_PARAM = DT_INVALID_PARAM;
        const BUFFER_TOO_SMALL = DT_BUFFER_TOO_SMALL;
        const OUT_OF_NODES = DT_OUT_OF_NODES;
        const PARTIAL_RESULT = DT_PARTIAL_RESULT;
        const ALREADY_OCCUPIED = DT_ALREADY_OCCUPIED;
    }
}

impl DtStatus {
    pub fn is_success(&self) -> bool {
        self.contains(DtStatus::SUCCESS)
    }

    pub fn is_in_progress(&self) -> bool {
        self.contains(DtStatus::IN_PROGRESS)
    }

    pub fn is_failed(&self) -> bool {
        self.contains(DtStatus::FAILURE)
    }
}

bitflags! {
    #[repr(transparent)]
    pub struct DtStraightPathFlags: u8 {
        const START = 0x01;
        const END = 0x02;
        const OFFMESH_CONNECTION = 0x03;
    }
}

#[link(name = "detour", kind = "static")]
extern "C" {
    pub fn dtNavMesh_alloc() -> *mut DtNavMesh;
    pub fn dtNavMesh_init(_self: *mut DtNavMesh, params: *const DtNavMeshParams) -> DtStatus;
    pub fn dtNavMesh_addTile(
        _self: *mut DtNavMesh,
        data: *mut u8,
        data_size: i32,
        flags: i32,
        last_ref: DtTileRef,
        result: *mut DtTileRef,
    ) -> DtStatus;
    pub fn dtNavMesh_free(_self: *mut DtNavMesh);

    pub fn dtQueryFilter_alloc() -> *mut DtQueryFilter;
    pub fn dtQueryFilter_free(_self: *mut DtQueryFilter);
    pub fn dtQueryFilter_setIncludeFlags(_self: *mut DtQueryFilter, include_flags: u16);
    pub fn dtQueryFilter_getIncludeFlags(_self: *mut DtQueryFilter) -> u16;
    pub fn dtQueryFilter_setExcludeFlags(_self: *mut DtQueryFilter, exclude_flags: u16);
    pub fn dtQueryFilter_getExcludeFlags(_self: *mut DtQueryFilter) -> u16;

    pub fn dtNavMeshQuery_alloc() -> *mut DtNavMeshQuery;
    pub fn dtNavMeshQuery_init(
        _self: *mut DtNavMeshQuery,
        dt_nav_mesh: *const DtNavMesh,
        max_nodes: i32,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_getPolyHeight(
        _self: *mut DtNavMeshQuery,
        poly_ref: DtPolyRef,
        position: *const DtVector,
        height: *mut f32,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_findNearestPoly(
        _self: *mut DtNavMeshQuery,
        center: *const DtVector,
        extents: *const DtVector,
        filter: *const DtQueryFilter,
        nearest_ref: *mut DtPolyRef,
        nearest_point: *mut DtVector,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_closestPointOnPoly(
        _self: *mut DtNavMeshQuery,
        poly_ref: DtPolyRef,
        position: *const DtVector,
        closest: *mut DtVector,
        position_over_poly: *mut bool,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_closestPointOnPolyBoundary(
        _self: *mut DtNavMeshQuery,
        poly_ref: DtPolyRef,
        position: *const DtVector,
        closest: *mut DtVector,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_findPath(
        _self: *mut DtNavMeshQuery,
        start_ref: DtPolyRef,
        end_ref: DtPolyRef,
        start_pos: *const DtVector,
        end_pos: *const DtVector,
        filter: *const DtQueryFilter,
        path: *mut DtPolyRef,
        path_count: *mut i32,
        max_path: i32,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_findStraightPath(
        _self: *mut DtNavMeshQuery,
        start_pos: *const DtVector,
        end_pos: *const DtVector,
        path: *const DtPolyRef,
        path_size: i32,
        straight_path_points: *mut DtVector,
        straight_path_flags: *mut DtStraightPathFlags,
        straight_path_polys: *mut DtPolyRef,
        straight_path_count: *mut i32,
        max_straight_path: i32,
        options: i32,
    ) -> DtStatus;
    pub fn dtNavMeshQuery_free(_self: *mut DtNavMeshQuery);
    pub fn dtNavMeshQuery_moveAlongSurface(
        _self: *mut DtNavMeshQuery,
        start_ref: DtPolyRef,
        start_pos: *const DtVector,
        end_pos: *const DtVector,
        filter: *const DtQueryFilter,
        result_pos: *mut DtVector,
        visited: *mut DtPolyRef,
        visited_count: *mut i32,
        max_visited_size: i32,
    ) -> DtStatus;
}
