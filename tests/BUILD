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

cc_test(name = "motor_bank_module_test",
        srcs = ["motor_bank_module_test.cc"],
        deps = [
            "//cc:module_dispatcher",
            "//cc:motor_bank_module",
            "@google_googletest//:gtest",
            "@google_googletest//:gtest_main",
            "@markstev_tensixty//tests:arduino_simulator"
        ])
