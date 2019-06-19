# Json File Guide
(See https://github.com/nlohmann/json for full documentation)
## Integration
- Within the BUILD.bazel file in the current workspace, add "@json" to the list of dependencies for each file using json.
    ```bazel
    cc_binary(
        name="your_file",
        srcs = ["your_file.cc"],
        deps = [
             "@json" <------Add this below other dependencies
        ]
    )
    ```
- Add the following code to the C++ file header:
  ```cpp
  #include <nlohmann/json.hpp>
  // for convenience
  using json = nlohmann::json;
  ```
- Add the json file to your current workspace.
- Use an input file stream to acces the json file data. Use the full file name relative to the dairlib folder.
  ```cpp
  std::ifstream joint_gains_file("folder1/folder2/file_name.json");
  ```
## File Parsing
