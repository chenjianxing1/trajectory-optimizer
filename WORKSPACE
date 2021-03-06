workspace(name = "trajectory_optimization")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")

http_archive(
  name = "gtest",
  url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
  sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
  build_file_content =
"""
cc_library(
    name = "main",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/gtest-all.cc"]
    ),
    hdrs = glob([
        "include/**/*.h",
        "src/*.h"
    ]),
    copts = ["-Iexternal/gtest/include"],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)
""",
  strip_prefix = "googletest-release-1.7.0",
)

http_archive(
    name = "pybind11",
    strip_prefix = "pybind11-2.3.0",
    urls = ["https://github.com/pybind/pybind11/archive/v2.3.0.zip"],
    build_file_content=
"""
cc_library(
    name = "pybind11",
    hdrs = glob([
        "include/**/**/*.h",
    ]),
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
    strip_include_prefix = "include/"
)
"""
)

new_local_repository(
    name = "python_linux",
    path = "./python/venv/",
    build_file_content = """
cc_library(
    name = "python-lib",
    srcs = glob(["lib/libpython3.*", "libs/python3.lib", "libs/python36.lib"]),
    hdrs = glob(["include/**/*.h", "include/*.h"]),
    includes = ["include/python3.6m", "include", "include/python3.7m", "include/python3.5m"], 
    visibility = ["//visibility:public"],
)
    """
)

http_archive(
  name = "com_github_eigen_eigen",
  sha256 = "dd254beb0bafc695d0f62ae1a222ff85b52dbaa3a16f76e781dce22d0d20a4a6",
  strip_prefix = "eigen-eigen-5a0156e40feb",
  urls = [
      "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2",
  ],
  build_file_content =
"""
cc_library(
    name = 'eigen',
    srcs = [],
    includes = ['.'],
    hdrs = glob(['Eigen/**']),
    visibility = ['//visibility:public'],
)
"""
)

git_repository(
  name = "com_github_nelhage_rules_boost",
  commit = "8a084196b14a396b6d4ff7c928ffbb6621f0d32c",
  remote = "https://github.com/patrickhart/rules_boost",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()


# ceres
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "6e16c8bc91b1310a44f3965e616383dbda48f83e8c1eaa2370a215057b00cabe",
    strip_prefix = "gflags-77592648e3f3be87d6c7123eb81cbad75f9aef5a",
    urls = [
        "https://mirror.bazel.build/github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
        "https://github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
    ]
)

http_archive(
  name = "com_github_google_glog",
  sha256 = "7083af285bed3995b5dc2c982f7de39bced9f0e6fd78d631f3285490922a0c3d",
  strip_prefix = "glog-3106945d8d3322e5cbd5658d482c9ffed2d892c0",
  urls = [
      "https://github.com/drigz/glog/archive/3106945d8d3322e5cbd5658d482c9ffed2d892c0.tar.gz",
  ]
)

new_git_repository(
  name = "com_google_ceres_solver",
  commit = "46ca461b7ee465c122c5122c8044ef11ec2298bc",
  remote = "https://github.com/ceres-solver/ceres-solver",
  build_file_content =
"""
load("//:bazel/ceres.bzl", "ceres_library")
ceres_library(
    name = "ceres",
    restrict_schur_specializations = True,
)
"""
)

