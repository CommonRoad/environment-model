//
// Created by sebastian on 26.12.20.
//

#include "test_command_line_input.h"

TEST_F(CommandLineInputTest, ReadCommandLineValuesValidDefault){
    int num_threads;
    std::string xmlFilePath;
    int argc { 1 };
    char **argv { };
    int error_code { CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath) };

    EXPECT_EQ(error_code, 0);
    EXPECT_EQ(num_threads, 1);
    EXPECT_EQ(xmlFilePath, "../test_scenarios/USA_Lanker-1_1_T-1.xml");
}

TEST_F(CommandLineInputTest, ReadCommandLineValuesValidParameters){
    int num_threads;
    std::string xmlFilePath;
    int argc { 5 };
    const char *array[5] {"myprogram", "--input-file", "test", "--t", "4" };
    char **argv {const_cast<char **>(array)};
    int error_code { CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath) };

    EXPECT_EQ(error_code, 0);
    EXPECT_EQ(num_threads, 4);
    EXPECT_EQ(xmlFilePath, "test");
}

TEST_F(CommandLineInputTest, ReadCommandLineValuesHelp){
    int num_threads;
    std::string xmlFilePath;
    int argc { 2 };
    const char *array[5] {"myprogram", "--help" };
    char **argv {const_cast<char **>(array)};
    int error_code { CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath) };

    EXPECT_EQ(error_code, 0);
}

TEST_F(CommandLineInputTest, ReadCommandLineValuesWrongNumberArguments){
    int num_threads;
    std::string xmlFilePath;
    int argc { 4 };
    const char *array[5] {"myprogram", "--help" };
    char **argv {const_cast<char **>(array)};
    int error_code { CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath) };

    EXPECT_EQ(error_code, 1);
}

TEST_F(CommandLineInputTest, ReadCommandLineValuesUnrecognizedOption){
    int num_threads;
    std::string xmlFilePath;
    int argc { 2 };
    const char *array[5] {"myprogram", "--abc" };
    char **argv {const_cast<char **>(array)};
    int error_code { CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath) };

    EXPECT_EQ(error_code, 1);
}

TEST_F(CommandLineInputTest, ReadCommandLineValuesMissingOption){
    int num_threads;
    std::string xmlFilePath;
    int argc { 2 };
    const char *array[5] {"myprogram", "--input-file" };
    char **argv {const_cast<char **>(array)};
    int error_code { CommandLine::readCommandLineValues(argc, argv, num_threads, xmlFilePath) };
    EXPECT_EQ(error_code, 1);
}