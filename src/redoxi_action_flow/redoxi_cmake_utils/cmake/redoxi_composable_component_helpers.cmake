find_package(rclcpp_components REQUIRED)

#[=======================================================================[.rst:
create_composable_components
---------------------------

Creates and registers ROS2 composable components from a list of plugins.

.. command:: create_composable_components

  .. code-block:: cmake

    create_composable_components(<ProjectName>
      SOURCE_FILE source_file1 [source_file2 ...]
      PLUGINS plugin1 [plugin2 ...]
      EXECUTABLES exec1 [exec2 ...]
    )

Creates a library containing composable components and registers each plugin as a
ROS2 component node with an associated standalone executable.

Arguments
^^^^^^^^^

``ProjectName``
  Name of the project, used to generate default names

``SOURCE_FILE``
  Required list of source files containing component implementations.

``PLUGINS``
  List of plugin class names to register as components

``EXECUTABLES`` 
  Required list of executable names corresponding to each plugin.
  Must have same length as PLUGINS list.

The macro:
- Creates a library named ${ProjectName}_components
- Links it to the main project library
- Registers each plugin as a ROS2 component
- Creates standalone executables for each component
- Installs the components library to the appropriate destinations
#]=======================================================================]
macro(create_composable_components ProjectName)
  cmake_parse_arguments(ARG "" "" "SOURCE_FILE;PLUGINS;EXECUTABLES" ${ARGN})
  
  # Validate required arguments
  if(NOT DEFINED ARG_SOURCE_FILE)
    message(FATAL_ERROR "SOURCE_FILE argument is required")
  endif()

  if(NOT DEFINED ARG_PLUGINS)
    message(FATAL_ERROR "PLUGINS argument is required")
  endif()
  
  if(NOT DEFINED ARG_EXECUTABLES)
    message(FATAL_ERROR "EXECUTABLES argument is required")
  endif()

  # Validate lengths match
  list(LENGTH ARG_PLUGINS num_plugins)
  list(LENGTH ARG_EXECUTABLES num_executables)
  if(NOT num_plugins EQUAL num_executables)
    message(FATAL_ERROR "Number of PLUGINS (${num_plugins}) must match number of EXECUTABLES (${num_executables})")
  endif()

  # Create components library
  add_library(${ProjectName}_components ${ARG_SOURCE_FILE})
  target_link_libraries(${ProjectName}_components PUBLIC ${ProjectName})

  # Register each plugin
  math(EXPR last_idx "${num_plugins} - 1")
  
  foreach(idx RANGE ${last_idx})
    list(GET ARG_PLUGINS ${idx} plugin)
    list(GET ARG_EXECUTABLES ${idx} exec_name)
    
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