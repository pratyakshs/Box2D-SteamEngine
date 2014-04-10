.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen
######################################
# Project Name (generate executable with this name)
TARGET = cs296_09_exe

SHARED_LIB = TRUE

# Project Paths
PROJECT_ROOT := $(CURDIR)
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/myobjs
BINDIR = $(PROJECT_ROOT)/mybins
DOCDIR = $(PROJECT_ROOT)/doc
INSTALLDIR = $(PROJECT_ROOT)
IMAGEDIR = $(PROJECT_ROOT)/images

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =-g -O3 -Wall -fno-strict-aliasing
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+= -L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib

######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)


.PHONY: all setup doc clean distclean exe static dynamic exelib mylibs

all:setup doc

setup:
	@$(ECHO) "Setting up compilation..."
	@mkdir -p myobjs
	@mkdir -p mybins
	@if [test -e $(PROJECT_ROOT)/external/lib/libBox2D.a] && [test -e $(EXTERNAL_ROOT)/include/Box2D];\
	then printf "Box2D is already installed\n"; \
	else cd $(PROJECT_ROOT)/external/src;tar xvzf Box2D.tgz ;pwd;cd Box2D;mkdir build296 ;cd build296 ; \
	cmake ../  ;\
	make -s;make -s install;fi;

exe : setup $(OBJS)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir cs296_09_$@)"
	@$(CC) -o $(BINDIR)/cs296_09_$@ $(LDFLAGS) $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

-include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -fPIC -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err

doc:
	@$(ECHO) -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@$(ECHO) "Done"

clean:
	@$(ECHO) -n "Cleaning up..."
	@$(RM) -rf $(BINDIR) $(LIBDIR) $(OBJDIR)
	

distclean: clean
	@$(RM) -rf $(BINDIR) $(DOCDIR)/html
	@$(RM) -rf $(EXTERNAL_ROOT)/src/Box2D
	@$(RM) -rf $(EXTERNAL_ROOT)/lib/libBox2D.a
	@$(RM) -rf $(EXTERNAL_ROOT)/include/Box2D

dist: distclean
	@cd ../../;tar cvzf cs296-g09-project.tar.gz cs296-g09-project 

install:exe
	@cd $(INSTALLDIR);mkdir -p steam_engine;
	@cp -r $(BINDIR) $(INSTALLDIR)/steam_engine
	@cp -r $(DOCDIR) $(INSTALLDIR)/steam_engine
	@cp -r $(IMAGEDIR) $(INSTALLDIR)/steam_engine
	@make clean 
