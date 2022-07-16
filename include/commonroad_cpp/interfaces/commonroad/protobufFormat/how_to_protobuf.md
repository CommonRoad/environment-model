# HOW TO PROTOBUF

The definition files can be found in `include/commonroad_cpp/interfaces/commonroad/protobufFormat/definitionFiles/`. The file
`commonroad.proto` contains the root of the whole message and imports all submessages which are divided
and stored in multiple files with ending `.proto`.
After modifying the definition files the corresponding code has to be generated immediately to be consistent.

The following command can be used to update the source and header files:
```
protoc -I=./include/commonroad_cpp/interfaces/commonroad/protobufFormat/definitionFiles/ --cpp_out=./include/commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses  ./include/commonroad_cpp/interfaces/commonroad/protobufFormat/definitionFiles/*.proto
```
Code for serializing and deserializing can be generated based on the previous mentioned
definition files. The code files are stored in `include/commonroad_cpp/interfaces/commonroad/protobufFormat/generatedClasses`.
The source files must be moved to the corresponding folder under src/.

Maybe some include paths have to be adjusted.
We recommend to use libprotoc 3.6.1 for generating the code (you can check the version via ```protoc --version```).

