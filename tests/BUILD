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
  name = "py_basics_tests",
  srcs = ["py_basics_tests.py"],
  data = ["//python:optimizer.so"],
  imports = ["../python/"]
)