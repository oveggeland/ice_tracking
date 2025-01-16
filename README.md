## Dependencies

### Open3D
I had issues with Open3D assigning small tasks to multiple processes, effectively increasing the runtime due to overhead. To avoid this, please build open3d with the following CMake flag -DWITH_OPENMP=OFF.