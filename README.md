hbba_base - Hybrid Behavior-Based Architecture base modules
===========================================================

This set of packages contains base modules for HBBA: ROS messages and tools for
interfacing with the Intention Workspace.
They have no external dependencies beyond ROS base modules such as topic_tools.

The goal of this set of packages is to remove large external dependencies pulled
in by the iw_translator (mainly or-tools) and script_engine packages, which
should make using HBBA easier to use from a remote system.

For the full distribution, see https://github.com/francoisferland/HBBA.

