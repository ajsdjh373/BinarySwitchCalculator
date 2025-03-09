# Thumper

This is a calculator for determing which standard resistor values can be used to make 2 SPDT switches connected to 4 resistors into a binary register able to generate 4 user specified outputs. It is assumed that the output from the switch-resistor register is taken from a resistor divider. This creates a system of 5 non-linear equations. This calculator solves it with brute force - trying every possible option and calculating the error from the target for each option.

## Project Goals

The high level functions provided should include:
- Standard 5% and 1% tolerance resistors values are hard coded per decade, with an algorithm to generate them for each decade needed from 10^2 through 10^5 bases
- Multi threading is used to accelerate calculations
- To conserve memory, any resistor combination that exceeds a specified error in % is not saved
- To conserve memory, any resistor combination that exceeds the minimum or maximum current bounds is not saved
- Resistor combinations that meet the criteria are sorted from least to most error
- This program runs on a windows terminal, and that's where the outputs are shown on the windows terminal

Some technical limitations are accepted up front:
- Compatible with Windows 10 and 11 operating systems only

## Style guide

In the interest of keeping things simple, rules are kept to a minimum.
- Compiler warnings are not allowed and must be addressed
- Do not inherit multiple layers deep, ie no X inherits from Y which inherits from Z, only X inherits from Y
- Do not use templates except as already implemented with C++ standard library functions
- Heap memory allocation must be done using standard library functions. The keywords "new", "malloc", "calloc", and "realloc" are forbidden.
- Prefer c++ references over raw pointers

Rules for comments and formatting are specified in STYLE.md.

## Building

The project uses Visual Studio 2022.
