GNSS / INS Integrated Navigation — C++ Migration
=================================================

.. image:: https://img.shields.io/badge/version-2.0--cpp-blue
   :alt: version

.. image:: https://img.shields.io/badge/license-BSD-green
   :alt: license

.. image:: https://img.shields.io/badge/standard-C%2B%2B17-orange
   :alt: C++17

----

Developer reference and step-by-step coding tutorial for porting the
Groves (2013) navigation simulation suite from
MATLAB to a clean, commented, production-ready C++17 library using **Eigen3**.

   *"Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems"*
   — Paul D. Groves, 2nd Edition. Original MATLAB software: BSD License, © 2012.

.. toctree::
   :maxdepth: 2
   :caption: Getting Started

   Source Code (GitHub) <https://github.com/ldv-1000111/gnss_ins_cpp>

   getting_started/overview
   getting_started/architecture
   getting_started/motion_profiles

.. toctree::
   :maxdepth: 2
   :caption: HPE Server Deployment

   deployment/hpe_server
   deployment/docker_workflow

.. toctree::
   :maxdepth: 2
   :caption: C++ Coding Tutorial

   tutorial/step1_scaffolding
   tutorial/step2_earth_math
   tutorial/step3_nav_equations
   tutorial/step4_imu_gnss
   tutorial/step5_kalman_filters
   tutorial/step6_sim_loops
   tutorial/step7_cli_docs

.. toctree::
   :maxdepth: 2
   :caption: Reference

   reference/function_reference
   reference/work_plan
   reference/constants_types
   reference/appendices

Indices
-------

* :ref:`genindex`
* :ref:`search`
