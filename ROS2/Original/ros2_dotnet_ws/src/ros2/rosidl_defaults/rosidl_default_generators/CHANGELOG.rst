^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rosidl_default_generators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.5.0 (2023-02-13)
------------------
* add service_msgs depend (`#24 <https://github.com/ros2/rosidl_defaults/issues/24>`_)
* [rolling] Update maintainers - 2022-11-07 (`#25 <https://github.com/ros2/rosidl_defaults/issues/25>`_)
* Contributors: Audrow Nash, Brian

1.4.0 (2022-09-08)
------------------
* Move dependencies to rosidl_core and depend on action_msgs (`#22 <https://github.com/ros2/rosidl_defaults/issues/22>`_)
  Move implementation to new packages rosidl_core_generators and rosidl_runtime_generators
  The new packages are located in a separate repository: https://github.com/ros2/rosidl_core.git
  rosidl_defaults now depends on the new packages, plus message definitions required for Actions (namely action_msgs).
  This allows users to avoid having to explictly depend on action_msgs.
* Contributors: Jacob Perron

1.3.0 (2022-05-04)
------------------

1.2.0 (2022-03-31)
------------------
* Unroll group dependencies (`#20 <https://github.com/ros2/rosidl_defaults/issues/20>`_)
* Contributors: Shane Loretz

1.1.1 (2021-04-06)
------------------

1.1.0 (2021-03-18)
------------------
* Update maintainers (`#13 <https://github.com/ros2/rosidl_defaults/issues/13>`_)
* Contributors: Shane Loretz

1.0.0 (2020-05-26)
------------------

0.9.0 (2020-04-25)
------------------

0.8.0 (2019-09-26)
------------------

0.7.0 (2019-04-14)
------------------

0.6.0 (2018-11-16)
------------------
* update manifest to adhere to tag order in schema (`#3 <https://github.com/ros2/rosidl_defaults/issues/3>`_)
* Contributors: Dirk Thomas

0.5.0 (2018-06-24)
------------------
* Merge pull request `#26 <https://github.com/ros2/rosidl_defaults/issues/26>`_ from ros2/remove-direct-vendor-typesupport-deps
* add and use groups for generator and runtime packages (`#25 <https://github.com/ros2/rosidl_defaults/issues/25>`_)
* Merge pull request `#22 <https://github.com/ros2/rosidl_defaults/issues/22>`_ from ros2/use_typesupport_group
* use ament index to collect typesupport packages
* add group_depends for typesupport
* Contributors: Dirk Thomas, Esteve Fernandez, Jackie Kay, Mikael Arguedas, Steven! Ragnarök, William Woodall
