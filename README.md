# Point Cloud Registration
Unipd - 3D data processing - Point Cloud Registration

## Usage
The implementation has to be done in C++ by using PCL or Open3D libraries.

Compilation instruction:
```
mkdir build && cd build
cmake ..
make
```
Remember to adjust the cmake accordingly if you intend to use PCL.
To execute:

```./registration path/to/source path/to/target path/to/output_transformation_matrix path/to/fused_cloud ```

## Results
![Capture](https://user-images.githubusercontent.com/58342884/177181693-9db2ef27-409f-4db1-931c-26da5b9ea730.PNG)
