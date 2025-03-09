# Formatting

## Filenames
Filenames depend on the module they belong to. Modules are indicated by a uniform 3 digit, all caps prefix followed by an underscore before the rest of the filename. The filename (excluding the extension) is the same for .h files and the matching .cpp files. All .cpp files must have a matching .h file, but not all .h files must have a matching .cpp file.

The exceptions to this are:
- File types other than .h and .cpp
- Main.cpp

## For header files
Comment to appear at the beginning of .h files:
/*
Author: Nathan Dunn
Module: module name

Functional description and intended use

Known bugs and limitations:
- a
- b
- c
*/

## For .cpp files
Comment to appear at the beginning of .cpp files:
/*
Author: Nathan Dunn
Module: module name

See associated header file for more information
*/

## Include statements
Include in the order of:
1. windows includes (WinWrapper.h)
2. C++ standard libraries
3. other external dependencies
4. internal dependencies originating from a module other than the module the file belongs to
5. in module dependencies

## Namespaces, classes, and functions
Forward declare classes at the beginning of every header file

Use the namespace for that module:
namespace EX
{
	class EXClass; 
}

// every class declares members in a specific order: public types, private types, public member functions, private member functions, public variables, private variables
class EX::EXClass
{
public:
	// public member types

private:
	// private member types

public:
	// public member functions
	// constructors first, then destructors, then everything else

private:
	// private member functions

public:
	// public member variables

private:
	// private member variables

};

// before functions
/*
Description of the function and its intended use

Arguments:
a		description
b		description
c		description

Return:
a		description

Safeties and known issues:
- a
- b
- c

*/

## Object and function naming
/*
For variables:
firstSecond_modifier

For functions, name spaces, enums, classes:
FirstSecond_Modifier

Acronyms and shortened words are always all caps

Physical units are separated by an underscore and capitalized per standard notation for the associated field
Units should be spelled out and treated as words if not paired with an hinting word
Compount units should be avoided in variable names, but if they must be joined use _ for multiplication and __ for division, eg kg_m__s2
time_ms		okay, probably milliseconds
force_MN	okay, MN is a common notation for meganewtons, which are used to measure force
ship_MN		not okay, it isn't obvious what MN could be
force_mn	not okay, m is usually representing milli, not mega, and n represents nano, not newton
Standard notation of prefixes is:
f	fempto	10e-15
p	pico	10e-12
n	nano	10e-9
u	micro	10e-6
m	milli	10e-3
c	centi	10e-2
h	hecto	10e2
k	kilo	10e3
M	mega	10e6
G	giga	10e9
T	tera	10e12
P	peta	10e15
E	exa		10e18
Z	zeta	10e21
Standard units are:
time		----------
s		second
yr		year
distance	----------
m		meter
in		inch
ft		foot
mi		mile
mil		1/1000 of an inch
micron	don't use microns, use um
mass		----------
lbm		pound mass
g		gram
slg		slug
force		----------
lb		pound force
N		newton
tn		short ton, 2000 lb
lt		long ton, 2240 lb
t		tonne, 1000 kg
kip		1000 lb (klb)
energy		----------
J		joule
Wh		watt hour
btu		british thermal unit
ftlb	foot pounds (allowable shorthand for ft_lb as it is widely used)
power		----------
hp		horse power
w		watt
Etc... add any new units here as they are used in the code!


*/
