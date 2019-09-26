cc_test(
  name = "parameter_tests",
  srcs = ["parameter_tests.cc"],
  copts = ["-Iexternal/gtest/include"],
  deps = [
    "//src/commons:parameters",
    "@gtest//:main"
  ],
	visibility = ["//visibility:public"]
)

cc_test(
  name = "dynamics_tests",
  srcs = ["dynamics_tests.cc"],
  copts = ["-Iexternal/gtest/include"],
  deps = [
    "//src/commons:parameters",
    "//src/dynamics:dynamics",
    "@gtest//:main"
  ],
	visibility = ["//visibility:public"]
)

cc_test(
  name = "optimizer_tests",
  srcs = ["optimizer_tests.cc"],
  copts = ["-Iexternal/gtest/include"],
  deps = [
    "//src/commons:parameters",
    "//src/dynamics:dynamics",
    "//src:optimizer",
    "@gtest//:main"
  ],
	visibility = ["//visibility:public"]
)

  
py_test(
  name = "py_optimizer_single_track_tests",
  srcs = ["py_optimizer_single_track_tests.py"],
  data = ["//python:optimizer.so"],
  imports = ["../python/"]
)

py_test(
  name = "py_commons_tests",
  srcs = ["py_commons_tests.py"],
  data = ["//python:optimizer.so"],
  imports = ["../python/"]
)