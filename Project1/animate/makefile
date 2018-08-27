
CFLAGS = -I$(XDIR)

XDIR = /usr/openwin/include
XLIB = /usr/openwin/lib

LIBS   = -lxview -lolgx -lX11 -lm

CFILES = animate.c vector_math.c nrutil.c
OFILES = animate.o vector_math.o nrutil.o
HFILES = vector_math.h

all: animate
animate: $(OFILES)
	cc -o $@ ${CFLAGS} -I$(XDIR) $(OFILES) -L$(XLIB) ${LIBS} 

# the name animate corresponds to the final compiled obj name like 'a.out'
# the $(name) replaces with the value of the variable
# $@ is the name animate

$(OFILES): $(HFILES)
 
# the above line means if the hfiles have been changed recompile the
# all the ofiles even if they haven't been changed
 

clean:
	rm $(OFILES) animate
lint:
	lint $(CFILES)
