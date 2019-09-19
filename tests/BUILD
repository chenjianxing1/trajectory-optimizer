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

cc_test(
  name = "geometry_tests",
  srcs = ["geometry_tests.cc"],
  copts = ["-Iexternal/gtest/include"],
  deps = [
    "//src/geometry:geometry",
    "@gtest//:main"
  ],
	visibility = ["//visibility:public"]
)
