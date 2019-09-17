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