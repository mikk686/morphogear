cmake_minimum_required(VERSION 3.5)

project(morphogear_dotnet)

find_package(ament_cmake REQUIRED)
find_package(rcldotnet REQUIRED)

find_package(rcldotnet_common REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(test_msgs REQUIRED)
find_package(rcldotnet REQUIRED)

find_package(dotnet_cmake_module REQUIRED)

set(CSHARP_TARGET_FRAMEWORK "netcoreapp6.0")
find_package(DotNETExtra REQUIRED)

set(_assemblies_dep_dlls
    ${rcldotnet_common_ASSEMBLIES_DLL}
    ${rcldotnet_ASSEMBLIES_DLL}
    ${builtin_interfaces_ASSEMBLIES_DLL}
    ${std_msgs_ASSEMBLIES_DLL}
    ${std_srvs_ASSEMBLIES_DLL}
    ${test_msgs_ASSEMBLIES_DLL}
)

add_dotnet_executable(communicatordotnet
  CommunicatorDotNet.cs
  INCLUDE_DLLS
  ${_assemblies_dep_dlls}
)


install_dotnet(communicatordotnet DESTINATION lib/${PROJECT_NAME}/dotnet)


ament_package()
