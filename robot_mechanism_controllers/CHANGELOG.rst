^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package robot_mechanism_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.17 (2019-08-08)
--------------------
* Remove actionlib typedefs defined in actionlib 1.12.0 (`#396 <https://github.com/PR2/pr2_controllers/issues/396>`_)
  * check actinlib_VERSION to removing re-typedef ResultPtr and FeedbackPtr https://github.com/ros/actionlib/issues/106
  * Remove actionlib typedefs defined upstream
    Signed-off-by: Shane Loretz <sloretz@osrfoundation.org>
* Contributors: Kei Okada, Shane Loretz

1.10.16 (2019-07-26)
--------------------
* Make sure to include the correct boost libraries.
  This follows the principle of "include what you use", and
  also should in theory fix the problems on the build farm.
  (`#394 <https://github.com/PR2/pr2_controllers/issues/394>`_)
  Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Contributors: Chris Lalancette

1.10.15 (2018-09-13)
--------------------

1.10.14 (2018-02-13)
--------------------
* Merge pull request `#388 <https://github.com/PR2/pr2_controllers/issues/388>`_ from k-okada/add_missing_dep
  add missing dependency
* Merge pull request `#387 <https://github.com/PR2/pr2_controllers/issues/387>`_ from k-okada/maintain
  change maintainer to ROS orphaned package maintainer
* robot_mechanism_controllers: add missing deps
* change maintainer to ROS orphaned package maintainer
* Contributors: Furushchev, Kei Okada

1.10.13 (2015-02-09)
--------------------
* Updated maintainership
* Contributors: dash

1.10.12 (2015-01-13)
--------------------

1.10.10 (2014-12-16)
--------------------

1.10.9 (2014-12-16)
-------------------
* Changelogs
* Added changelogs
* Changelogs; maintainership
* Contributors: TheDash

* Changelogs; maintainership
* Contributors: TheDash
