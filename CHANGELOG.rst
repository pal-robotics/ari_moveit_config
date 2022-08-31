^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package ari_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.9 (2022-08-31)
------------------
* Merge branch 'gallium-fix' into 'master'
  Add default_velocity_scaling_factor to avoid slow movements
  See merge request robots/ari_moveit_config!7
* Add default_velocity_scaling_factor to avoid slow movements
* Contributors: David ter Kuile, davidfernandez

0.0.8 (2022-06-09)
------------------
* Merge branch 'ari-v2' into 'master'
  Add controllers for additional caster joints in v2
  See merge request robots/ari_moveit_config!8
* Update srdf for ari-v2 with extra caster link
* Add controllers for additional caster joints in v2
* Contributors: David ter Kuile, Luca Marchionni, davidfernandez

0.0.7 (2022-05-30)
------------------
* Merge branch 'ari-v2' into 'master'
  update launch an config files for different robot_versions
  See merge request robots/ari_moveit_config!6
* Fix ompl launch file typo
* Fix typo in controller_v2.yaml
* disable collision checking for docking link
* remove redundant whiteline
* update launch an config files for different robot_versions
* Contributors: David ter Kuile, davidfernandez

0.0.6 (2021-07-30)
------------------
* Merge branch 'fixing_laser_model' into 'master'
  fixing laser_model set default false
  See merge request robots/ari_moveit_config!5
* Fixed laser_suffix that manages both cases when laser_model is bool and when is a string
* fixing laser_model set default false
* Contributors: antoniobrandi, saikishor

0.0.5 (2021-05-20)
------------------
* Merge branch 'ari_laser_new' into 'master'
  Ari laser new
  See merge request robots/ari_moveit_config!4
* Update launch/move_group.launch
* Update launch/move_group.launch
* Update launch/move_group.launch
* Deleted redundant
* Added laser ari srdf
* Added laser ari srdf
* Contributors: davidfernandez, sergiomoyano

0.0.4 (2020-03-24)
------------------
* Merge branch 'add-hands' into 'master'
  Add Hand joints to srdf
  See merge request robots/ari_moveit_config!3
* Add Hand joints to srdf
* Contributors: davidfernandez, jordanpalacios

0.0.3 (2020-01-17)
------------------
* Merge branch 'moveit_fix' into 'master'
  fix moveit cartesian goals issue on robot
  See merge request robots/ari_moveit_config!2
* fix moveit cartesian goals issue on robot
* Contributors: Sai Kishor Kothakota

0.0.2 (2019-11-14)
------------------
* Update ARI SRDF file
* Contributors: Sai Kishor Kothakota

0.0.1 (2019-11-12)
------------------
* fix the package.xml version
* Merge branch 'head_front' into 'master'
  changed the head_camera_link to head_front_camera_link
  See merge request robots/ari_moveit_config!1
* changed the head_camera_link to head_front_camera_link
* Initial commit
* Contributors: Sai Kishor Kothakota
