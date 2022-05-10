#include "../recastnavigation/Detour/Include/DetourStatus.h"
#include "../recastnavigation/Detour/Include/DetourNavMesh.h"
#include "../recastnavigation/Detour/Include/DetourNavMeshQuery.h"

extern "C"
{

  dtNavMesh *dtNavMesh_alloc()
  {
    return dtAllocNavMesh();
  }

  dtStatus dtNavMesh_init(dtNavMesh *mesh, const dtNavMeshParams *params)
  {
    return mesh->init(params);
  }

  dtStatus dtNavMesh_initSingle(dtNavMesh *mesh, unsigned char *data, int dataSize, int flags)
  {
    return mesh->init(data, dataSize, flags);
  }

  dtStatus dtNavMesh_addTile(dtNavMesh *mesh, unsigned char *data, int dataSize,
                             int flags, dtTileRef lastRef, dtTileRef *result)
  {
    return mesh->addTile(data, dataSize, flags, lastRef, result);
  }

  dtNavMeshQuery *dtNavMeshQuery_alloc()
  {
    return dtAllocNavMeshQuery();
  }

  void dtNavMesh_free(dtNavMesh *mesh)
  {
    return dtFreeNavMesh(mesh);
  }

  dtQueryFilter *dtQueryFilter_alloc()
  {
    return new dtQueryFilter();
  }

  void dtQueryFilter_free(dtQueryFilter *filter)
  {
    delete filter;
  }

  void dtQueryFilter_setIncludeFlags(dtQueryFilter *filter, unsigned short flags)
  {
    filter->setIncludeFlags(flags);
  }

  unsigned short dtQueryFilter_getIncludeFlags(dtQueryFilter *filter)
  {
    return filter->getIncludeFlags();
  }

  void dtQueryFilter_setExcludeFlags(dtQueryFilter *filter, unsigned short flags)
  {
    filter->setExcludeFlags(flags);
  }

  unsigned short dtQueryFilter_getExcludeFlags(dtQueryFilter *filter)
  {
    return filter->getExcludeFlags();
  }

  dtStatus dtNavMeshQuery_init(dtNavMeshQuery *query, dtNavMesh *mesh, int maxNodes)
  {
    return query->init(mesh, maxNodes);
  }

  dtStatus dtNavMeshQuery_getPolyHeight(dtNavMeshQuery *query, dtPolyRef polyRef, const float *pos, float *height)
  {
    return query->getPolyHeight(polyRef, pos, height);
  }

  dtStatus dtNavMeshQuery_findNearestPoly(dtNavMeshQuery *query, const float *center, const float *extents,
                                          const dtQueryFilter *filter,
                                          dtPolyRef *nearestRef, float *nearestPt)
  {
    return query->findNearestPoly(center, extents, filter, nearestRef, nearestPt);
  }

  dtStatus dtNavMeshQuery_closestPointOnPoly(dtNavMeshQuery *query, dtPolyRef ref, const float *pos, float *closest, bool *posOverPoly)
  {
    return query->closestPointOnPoly(ref, pos, closest, posOverPoly);
  }

  dtStatus dtNavMeshQuery_closestPointOnPolyBoundary(dtNavMeshQuery *query, dtPolyRef ref, const float *pos, float *closest)
  {
    return query->closestPointOnPolyBoundary(ref, pos, closest);
  }

  dtStatus dtNavMeshQuery_findPath(dtNavMeshQuery *query, dtPolyRef startRef, dtPolyRef endRef,
                                   const float *startPos, const float *endPos,
                                   const dtQueryFilter *filter,
                                   dtPolyRef *path, int *pathCount, const int maxPath)
  {
    return query->findPath(startRef, endRef, startPos, endPos, filter, path, pathCount, maxPath);
  }

  dtStatus dtNavMeshQuery_moveAlongSurface(dtNavMeshQuery *query, dtPolyRef startRef,
                                           const float *startPos, const float *endPos,
                                           const dtQueryFilter *filter,
                                           float *resultPos, dtPolyRef *visited, int *visitedCount, const int maxVisitedSize)
  {
    return query->moveAlongSurface(startRef, startPos, endPos, filter, resultPos, visited, visitedCount, maxVisitedSize);
  }

  dtStatus dtNavMeshQuery_findStraightPath(dtNavMeshQuery *query, const float *startPos, const float *endPos,
                                           const dtPolyRef *path, const int pathSize,
                                           float *straightPath, unsigned char *straightPathFlags, dtPolyRef *straightPathRefs,
                                           int *straightPathCount, const int maxStraightPath, const int options)
  {
    return query->findStraightPath(startPos, endPos, path, pathSize, straightPath, straightPathFlags, straightPathRefs, straightPathCount, maxStraightPath, options);
  }

  void dtNavMeshQuery_free(dtNavMeshQuery *query)
  {
    return dtFreeNavMeshQuery(query);
  }
};