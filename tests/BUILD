package(default_visibility = ["//visibility:public"])

cc_test(name = "module_dispatcher_test",
        srcs = ["module_dispatcher_test.cc"],
        deps = [
            "//cc:module_dispatcher",
            "@google_googletest//:gtest",
            "@google_googletest//:gtest_main",
        ])

cc_test(name = "motor_test",
        srcs = ["motor_test.cc"],
        deps = [
            "//cc:motor",
            "@google_googletest//:gtest",
            "@google_googletest//:gtest_main",
            "@markstev_tensixty//tests:arduino_simulator"
        ])
