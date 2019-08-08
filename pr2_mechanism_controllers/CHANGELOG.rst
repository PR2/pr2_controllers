^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pr2_mechanism_controllers
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.10.17 (2019-08-08)
--------------------

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
* Merge pull request `#390 <https://github.com/pr2/pr2_controllers/issues/390>`_ from k-okada/add_travis
  update travis.yml
* add fix for urdfmodel 1.0.0(melodic),
  since 12.04 have urdfmodel < 1.0.0, it will fail to compile on indigo, so we need to chaeck URDFDOM_version
* Contributors: Kei Okada

1.10.14 (2018-02-13)
--------------------
* Merge pull request `#389 <https://github.com/PR2/pr2_controllers/issues/389>`_ from k-okada/kinetic-devel
  - use Eigen3 instead of Eigen
* use Eigen3 instead of Eigen
* Merge pull request `#388 <https://github.com/PR2/pr2_controllers/issues/388>`_ from k-okada/add_missing_dep
  add missing dependency
* Merge pull request `#387 <https://github.com/PR2/pr2_controllers/issues/387>`_ from k-okada/maintain
  change maintainer to ROS orphaned package maintainer
* Merge pull request `#369 <https://github.com/PR2/pr2_controllers/issues/369>`_ from muratsevim/hydro-devel
  std namespace prefix is added to isnan calls
* mechanism_controllers: add missing pr2_msgs dependency
* change maintainer to ROS orphaned package maintainer
* std namespace prefix is added to isnan calls
* Contributors: Kei Okada, Mehmet Murat Sevim, Michael Görner

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
