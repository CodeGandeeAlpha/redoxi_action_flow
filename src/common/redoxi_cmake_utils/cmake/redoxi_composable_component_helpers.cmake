find_package(rclcpp_components REQUIRED)

#[=======================================================================[.rst:
create_composable_components
---------------------------

Creates and registers ROS2 composable components from a list of plugins.

.. command:: create_composable_components

  .. code-block:: cmake

    create_composable_components(<ProjectName>
      [SOURCE_FILE <source_file>]
      PLUGINS plugin1 [plugin2 ...]
      [EXECUTABLES exec1 [exec2 ...]]
    )

Creates a library containing composable components and registers each plugin as a
ROS2 component node with an associated standalone executable.

Arguments
^^^^^^^^^

``ProjectName``
  Name of the project, used to generate default names

``SOURCE_FILE``
  Optional path to the source file containing component implementations.
  Defaults to "src/${ProjectName}_components.cpp"

``PLUGINS``
  List of plugin class names to register as components

``EXECUTABLES`` 
  Optional list of executable names corresponding to each plugin.
  If not provided, names are generated from plugin names by replacing "::" with "_"
  and prefixing with "node_executable_"

The macro:
- Creates a library named ${ProjectName}_components
- Links it to the main project library
- Registers each plugin as a ROS2 component
- Creates standalone executables for each component
- Installs the components library to the appropriate destinations
#]=======================================================================]
macro(create_composable_components ProjectName)
  cmake_parse_arguments(ARG "" "SOURCE_FILE" "PLUGINS;EXECUTABLES" ${ARGN})
  
  # Set default source file if not provided
  if(NOT ARG_SOURCE_FILE)
    set(ARG_SOURCE_FILE "src/${ProjectName}_components.cpp")
  endif()

  # Create components library
  add_library(${ProjectName}_components ${ARG_SOURCE_FILE})
  target_link_libraries(${ProjectName}_components PUBLIC ${ProjectName})

  # Register each plugin
  list(LENGTH ARG_PLUGINS num_plugins)
  math(EXPR last_idx "${num_plugins} - 1")
  
  foreach(idx RANGE ${last_idx})
    list(GET ARG_PLUGINS ${idx} plugin)
    
    # Use provided executable name if available, otherwise generate one
    if(DEFINED ARG_EXECUTABLES)
      list(GET ARG_EXECUTABLES ${idx} exec_name)
    else()
      string(REPLACE "::" "_" exec_name "${plugin}")
      set(exec_name "node_executable_${exec_name}")
    endif()
    
    rclcpp_components_register_node(${ProjectName}_components
      PLUGIN "${plugin}"
      EXECUTABLE ${exec_name}
    )
  endforeach()

  # Install targets
  install(TARGETS ${ProjectName}_components
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endmacro()