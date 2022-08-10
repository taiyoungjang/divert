mod binding;

use std::{
    marker,
    ops::{Add, Mul, Sub},
};

pub use binding::DtStraightPathFlags;

use binding::*;
use thiserror::Error;

/// 3D Vector used in Recast Navigation, correspond to a [f32; 3]
/// This abstraction is provided to combat misunderstanding of point ordering
/// Recast expects y, z, x ordering while many applications use x, y, z ordering
pub type Vector = DtVector;

/// Provides functionality to initialize Vectors compatible with Recast
/// Additionally provides basic math functions used with 3D Vectors
impl Vector {
    /// Simple function to help with initializing maintaining expected ordering
    pub fn from_xyz(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Simple function to help with initializing maintaining expected ordering
    pub fn from_yzx(y: f32, z: f32, x: f32) -> Self {
        Self { x, y, z }
    }

    /// Dot product of the vector and other
    pub fn dot(&self, other: &Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
}

impl Add for Vector {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
            x: self.x + other.x,
            y: self.y + other.y,
            z: self.z + other.z,
        }
    }
}

impl Sub for Vector {
    type Output = Self;

    fn sub(self, other: Self) -> Self {
        Self {
            x: self.x - other.x,
            y: self.y - other.y,
            z: self.z - other.z,
        }
    }
}

impl Mul<f32> for Vector {
    type Output = Self;

    fn mul(self, scalar: f32) -> Self {
        Self {
            x: self.x * scalar,
            y: self.y * scalar,
            z: self.z * scalar,
        }
    }
}

/// Typedef to DtNavMeshParams
/// Affords the ability in future to add custom functionality
pub type NavMeshParams = DtNavMeshParams;

/// Typedef to DtPolyRef
/// Affords the ability in future to add custom functionality
pub type PolyRef = DtPolyRef;

/// Typedef to DtTileRef
/// Affords the ability in future to add custom functionality
pub type TileRef = DtTileRef;

#[derive(Error, Debug)]
pub enum DivertError {
    #[error("detour internal status failure `{0:?}")]
    Failure(DtStatus),
    #[error("detour unexpected null ptr failure")]
    NullPtr(),
    #[error("detour nav mesh unexpected dtNavMeshQuery::getPolyHeight failure `{0:?}`")]
    GetPolyHeightFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::findNearestPoly failure `{0:?}`")]
    FindNearestPolyFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::closestPointOnPoly failure `{0:?}`")]
    ClosestPointOnPolyFailure(DtStatus),
    #[error(
        "detour nav mesh unexpected dtNavMeshQuery::closestPointOnPolyBoundary failure `{0:?}`"
    )]
    ClosestPointOnPolyBoundaryFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::findPath failure `{0:?}`")]
    FindPathFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::findStraightPath failure `{0:?}`")]
    FindStraightPathFailure(DtStatus),
    #[error("detour nav mesh unexpected dtNavMeshQuery::moveAlongSurface failure `{0:?}`")]
    MoveAlongSurfaceFailure(DtStatus),
}

pub type DivertResult<T> = std::result::Result<T, DivertError>;

/// Safe bindings to dtNavMesh
/// Handles life time of the dtNavMesh and will release resources when dropped
pub struct NavMesh<'a> {
    handle: *mut DtNavMesh,
    _phantom: marker::PhantomData<&'a DtNavMesh>,
}

unsafe impl Send for NavMesh<'_> {}

/// Provides functionality to interact with NavMesh and its underlying dtNavMesh
impl<'a> NavMesh<'a> {
    /// Allocates and initializes a dtNavMesh for NavMesh to handle
    /// Errors if allocation returns a null pointer, or the dtNavMesh->init function returns a failed status
    pub fn new(nav_mesh_params: &NavMeshParams) -> DivertResult<Self> {
        let dt_nav_mesh = unsafe { dtNavMesh_alloc() };

        if dt_nav_mesh.is_null() {
            return Err(DivertError::NullPtr());
        }

        let init_status = unsafe { dtNavMesh_init(dt_nav_mesh, nav_mesh_params) };
        if init_status.is_failed() {
            return Err(DivertError::Failure(init_status));
        }

        Ok(Self {
            handle: dt_nav_mesh,
            _phantom: marker::PhantomData,
        })
    }

    /// Accepts a byte vector representing a dtTile, adding it to the inner dtNavMesh
    /// The byte vector is forgotten after being added to the dtNavMesh
    /// Forgetting the memory is critical, because the memory is now owned by the dtNavMesh
    pub fn add_tile(&mut self, input_data: Vec<u8>) -> DivertResult<TileRef> {
        let mut boxed_slice = input_data.into_boxed_slice();
        let data = boxed_slice.as_mut_ptr();
        let data_size = boxed_slice.len();

        let mut tile_ref = TileRef::default();
        let add_tile_status = unsafe {
            dtNavMesh_addTile(
                self.handle,
                data,
                data_size as i32,
                1,
                TileRef::default(),
                &mut tile_ref,
            )
        };

        if add_tile_status.is_failed() {
            return Err(DivertError::Failure(add_tile_status));
        }

        std::mem::forget(boxed_slice);
        Ok(tile_ref)
    }
}

/// Handles freeing the inner dtNavMesh
/// subsequently handles freeing the tile data added to this NavMesh
impl<'a> Drop for NavMesh<'a> {
    /// Frees dtNavMesh resources with dtFreeNavMesh
    fn drop(&mut self) {
        unsafe { dtNavMesh_free(self.handle) }
    }
}

/// Safe bindings to dtQueryFilter
/// Handles life time of the dtQueryFilter and will release resources when dropped
pub struct QueryFilter<'a> {
    handle: *mut DtQueryFilter,
    _phantom: marker::PhantomData<&'a DtQueryFilter>,
}

unsafe impl Send for QueryFilter<'_> {}

/// Provides functionality to interact with QueryFilter and its underlying dtQueryFilter
impl<'a> QueryFilter<'a> {
    /// Allocates a dtQueryFilter
    /// Errors if the allocation returns a null pointer
    pub fn new() -> DivertResult<Self> {
        let dt_query_filter = unsafe { dtQueryFilter_alloc() };
        if dt_query_filter.is_null() {
            Err(DivertError::NullPtr())
        } else {
            Ok(Self {
                handle: dt_query_filter,
                _phantom: marker::PhantomData,
            })
        }
    }

    /// Sets the filter's include flags
    pub fn set_include_flags(&mut self, include_flags: u16) {
        unsafe {
            dtQueryFilter_setIncludeFlags(self.handle, include_flags);
        }
    }

    /// Retrieves the filter's include flags
    pub fn get_include_flags(&self) -> u16 {
        unsafe { dtQueryFilter_getIncludeFlags(self.handle) }
    }

    /// Sets the filter's exclude flags
    pub fn set_exclude_flags(&mut self, exclude_flags: u16) {
        unsafe {
            dtQueryFilter_setExcludeFlags(self.handle, exclude_flags);
        }
    }

    /// Retrieves the filter's exclude flags
    pub fn get_exclude_flags(&self) -> u16 {
        unsafe { dtQueryFilter_getExcludeFlags(self.handle) }
    }
}

/// Handles freeing the inner dtQueryFilter
impl<'a> Drop for QueryFilter<'a> {
    fn drop(&mut self) {
        unsafe { dtQueryFilter_free(self.handle) }
    }
}

/// Safe bindings to dtNavMeshQuery
/// Handles life time of the dtNavMeshQuery and will release resources when dropped
pub struct NavMeshQuery<'a> {
    handle: *mut DtNavMeshQuery,
    _phantom: marker::PhantomData<&'a DtNavMeshQuery>,
}

unsafe impl Send for NavMeshQuery<'_> {}

/// Provides functionality to interact with NavMeshQuery and its underlying dtNavMeshQuery
impl<'a> NavMeshQuery<'a> {
    /// Allocates and initializes a dtNavMeshQuery for NavMeshQuery to handle
    /// Errors if allocation returns a null pointer, or the dtNavMeshQuery->init function returns a failed status
    pub fn new(nav_mesh: &NavMesh, max_nodes: i32) -> DivertResult<Self> {
        let dt_nav_mesh_query = unsafe { dtNavMeshQuery_alloc() };

        if dt_nav_mesh_query.is_null() {
            return Err(DivertError::NullPtr());
        }

        let init_status =
            unsafe { dtNavMeshQuery_init(dt_nav_mesh_query, nav_mesh.handle, max_nodes) };
        if init_status.is_failed() {
            return Err(DivertError::Failure(init_status));
        }

        Ok(Self {
            handle: dt_nav_mesh_query,
            _phantom: marker::PhantomData,
        })
    }

    /// Queries for polygon height given the reference polygon and position on the polygon
    /// Errors if ffi function returns a failed DtStatus
    pub fn get_poly_height(&self, poly_ref: PolyRef, position: &DtVector) -> DivertResult<f32> {
        let mut height: f32 = 0.0;

        let get_poly_height_status =
            unsafe { dtNavMeshQuery_getPolyHeight(self.handle, poly_ref, position, &mut height) };

        if get_poly_height_status.is_failed() {
            return Err(DivertError::GetPolyHeightFailure(get_poly_height_status));
        }

        Ok(height)
    }

    /// Queries for nearest polygon given a center point, a search area (extents), and a filter
    /// Errors if ffi function returns a failed DtStatus
    pub fn find_nearest_poly(
        &self,
        center: &Vector,
        extents: &Vector,
        filter: &QueryFilter,
    ) -> DivertResult<(PolyRef, Vector)> {
        let mut closest_point = Vector::default();
        let mut nearest_ref = PolyRef::default();

        let nearest_status = unsafe {
            dtNavMeshQuery_findNearestPoly(
                self.handle,
                center,
                extents,
                filter.handle,
                &mut nearest_ref,
                &mut closest_point,
            )
        };

        if nearest_status.is_failed() {
            return Err(DivertError::FindNearestPolyFailure(nearest_status));
        }

        Ok((nearest_ref, closest_point))
    }

    /// Queries for closest point on poly to a given position
    /// Errors if ffi function returns a failed DtStatus
    pub fn closest_point_on_poly(
        &self,
        poly_ref: PolyRef,
        position: &Vector,
    ) -> DivertResult<(Vector, bool)> {
        let mut closest_point = Vector::default();
        let mut position_over_poly = false;

        let nearest_status = unsafe {
            dtNavMeshQuery_closestPointOnPoly(
                self.handle,
                poly_ref,
                position,
                &mut closest_point,
                &mut position_over_poly,
            )
        };

        if nearest_status.is_failed() {
            return Err(DivertError::ClosestPointOnPolyFailure(nearest_status));
        }

        Ok((closest_point, position_over_poly))
    }

    /// Queries for closest point on poly boundary to a given position
    /// Errors if ffi function returns a failed DtStatus
    pub fn closest_point_on_poly_boundary(
        &self,
        poly_ref: PolyRef,
        position: &Vector,
    ) -> DivertResult<Vector> {
        let mut closest_point = Vector::default();

        let dt_result = unsafe {
            dtNavMeshQuery_closestPointOnPolyBoundary(
                self.handle,
                poly_ref,
                position,
                &mut closest_point,
            )
        };

        if dt_result.is_failed() {
            return Err(DivertError::ClosestPointOnPolyBoundaryFailure(dt_result));
        }

        Ok(closest_point)
    }

    /// Generates a polygon path from one (poly, position) to another (poly, position)
    /// Errors if ffi function returns a failed DtStatus
    pub fn find_path(
        &self,
        start_ref: PolyRef,
        end_ref: PolyRef,
        start_pos: &Vector,
        end_pos: &Vector,
        filter: &QueryFilter,
        max_path: i32,
    ) -> DivertResult<Vec<PolyRef>> {
        let mut path_count = 0;
        let mut path: Vec<PolyRef> = Vec::with_capacity(max_path.try_into().unwrap());

        let find_path_status = unsafe {
            dtNavMeshQuery_findPath(
                self.handle,
                start_ref,
                end_ref,
                start_pos,
                end_pos,
                filter.handle,
                path.as_mut_ptr(),
                &mut path_count,
                max_path,
            )
        };

        log::trace!("FindPathStatus: {:#?}", find_path_status);

        if find_path_status.is_failed() {
            return Err(DivertError::FindPathFailure(find_path_status));
        }

        unsafe {
            path.set_len(path_count as usize);
        }

        Ok(path)
    }

    /// Generates a (poly, position) path from on (poly, position) to another (poly, position)
    /// Errors if ffi function returns a failed DtStatus
    pub fn find_straight_path(
        &self,
        start_pos: &Vector,
        end_pos: &Vector,
        poly_path: &[PolyRef],
        max_path: i32,
        options: i32,
    ) -> DivertResult<Vec<(Vector, DtStraightPathFlags, PolyRef)>> {
        let mut straight_path_count = 0;
        let mut straight_path_points: Vec<DtVector> =
            Vec::with_capacity(max_path.try_into().unwrap());
        let mut straight_path_flags: Vec<DtStraightPathFlags> =
            Vec::with_capacity(max_path.try_into().unwrap());
        let mut straight_path_polys: Vec<PolyRef> =
            Vec::with_capacity(max_path.try_into().unwrap());

        let find_path_status = unsafe {
            dtNavMeshQuery_findStraightPath(
                self.handle,
                start_pos,
                end_pos,
                poly_path.as_ptr(),
                poly_path.len().try_into().unwrap(),
                straight_path_points.as_mut_ptr(),
                straight_path_flags.as_mut_ptr(),
                straight_path_polys.as_mut_ptr(),
                &mut straight_path_count,
                max_path,
                options,
            )
        };

        log::trace!("FindStraightPathStatus: {:#?}", find_path_status);

        if find_path_status.is_failed() {
            return Err(DivertError::FindStraightPathFailure(find_path_status));
        }

        let path_count = straight_path_count as usize;

        unsafe {
            straight_path_points.set_len(path_count);
            straight_path_flags.set_len(path_count);
            straight_path_polys.set_len(path_count);
        }

        let path_result = straight_path_points
            .into_iter()
            .zip(straight_path_flags.into_iter())
            .zip(straight_path_polys.into_iter())
            .map(|((pos, flags), poly_ref)| (pos, flags, poly_ref))
            .collect();

        Ok(path_result)
    }

    /// Generates a poly path while moving from (poly, position) to a (poly)
    /// Errors if ffi function returns a failed DtStatus
    pub fn move_along_surface(
        &self,
        start_ref: PolyRef,
        start_pos: &Vector,
        end_pos: &Vector,
        filter: &QueryFilter,
        max_visit: i32,
    ) -> DivertResult<(Vector, Vec<PolyRef>)> {
        let mut visited_count = 0;
        let mut visited: Vec<PolyRef> = Vec::with_capacity(max_visit.try_into().unwrap());
        let mut result_pos = Vector::default();

        let move_along_surface_result = unsafe {
            dtNavMeshQuery_moveAlongSurface(
                self.handle,
                start_ref,
                start_pos,
                end_pos,
                filter.handle,
                &mut result_pos,
                visited.as_mut_ptr(),
                &mut visited_count,
                max_visit,
            )
        };

        if move_along_surface_result.is_failed() {
            return Err(DivertError::MoveAlongSurfaceFailure(
                move_along_surface_result,
            ));
        }

        unsafe {
            visited.set_len(visited_count as usize);
        }

        Ok((result_pos, visited))
    }
}

/// Handles freeing the inner dtNavMeshQuery
impl<'a> Drop for NavMeshQuery<'a> {
    /// Frees dtNavMeshQuery resources with dtFreeNavMeshQuery
    fn drop(&mut self) {
        unsafe { dtNavMeshQuery_free(self.handle) }
    }
}

#[cfg(test)]
mod tests {

    use crate::{NavMesh, NavMeshParams, NavMeshQuery, QueryFilter};

    #[test]
    fn test_nav_mesh() {
        let nav_mesh_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 32.0,
            tile_height: 32.0,
            max_polys: 1000,
            max_tiles: 1,
        };

        let nav_mesh = NavMesh::new(&nav_mesh_params);
        assert!(nav_mesh.is_ok());

        let _nav_mesh = nav_mesh.unwrap();
    }

    #[test]
    fn test_nav_mesh_query() {
        let nav_mesh_params = NavMeshParams {
            origin: [0.0, 0.0, 0.0],
            tile_width: 32.0,
            tile_height: 32.0,
            max_polys: 1000,
            max_tiles: 1,
        };

        let nav_mesh = NavMesh::new(&nav_mesh_params).unwrap();
        let nav_mesh_query = NavMeshQuery::new(&nav_mesh, 100);
        assert!(nav_mesh_query.is_ok());
    }

    #[test]
    fn test_query_filter() {
        let filter = QueryFilter::new();
        assert!(filter.is_ok());

        let mut filter = filter.unwrap();

        filter.set_include_flags(1);
        assert_eq!(filter.get_include_flags(), 1);

        filter.set_exclude_flags(1);
        assert_eq!(filter.get_exclude_flags(), 1);
    }
}
