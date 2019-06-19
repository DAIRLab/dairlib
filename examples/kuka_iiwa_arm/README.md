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
  std::ifstream data_file("folder1/folder2/file_name.json");
  ```
## Json File Example
- The nlohmann package may be used to parse a variety of data ypes from a given json file.
- See below example for specific file syntax:
  ```json
  {
    "pi": 3.141,
    "gravity_comp": true,
    "name": "5895",
    "nothing": null,
    "Gains": {
        "P": 30000,
        "I": 0
    },                    
    "list": [1, 0, 2],
    "object": {
        "currency": "USD",
        "value": 42.99
    }
  }
  ```
- Example Notes:
  - Stores value 3.141 to key "pi"
  - Stores bool "true" to key "gravity-comp"
  - Stores string "5895" to key "name"
  - Stores value "null" to key "nothing"
  - Stores two values under "Gains" object:
    - Assigns 30000 to "P" 
    - Assigns 0 to "I"
  - Stores list [1, 0, 2] under key "list"
  - Stores two values under "object" object:
    - Assigns string "USD" to key "currency"
    - Assigns 42.99 to key "value"
  
## Json Parsing
- First, initialize a json object and deserialize the json file using the json::parse(ifstream) function,
  ```cpp
  json data = json::parse(data_file);
  ```
- **or** the overloaded >> operator.
  ```cpp
  json data;
  data_file >> data;
  ```
- Once the json object has been initialized, the json file values are accessible as follows:
  ```cpp
  const float PI = data["pi"];
  bool gravity_compensation = data["gravity_comp"];
  std::string name = data["name"];
  std::string noVal = data["nothing"];
  const double P = data["Gains"]["P"]; //Accesses "P" within "Gains" object
  int firstVal = data["list"][0]; //Accesses first element in "list"
  int ca$hMoney = data["object"]["value"];
  ```
Visit https://github.com/nlohmann/json for additional functionality.