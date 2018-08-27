
CFLAGS =

LIBS   =

CFILES = paint_gen.c
OFILES = paint_gen.o 
HFILES = vector_math.h

all: paint_gen
paint_gen: $(OFILES)
	cc -o $@ ${CFLAGS} $(OFILES) ${LIBS} 

# the name paint_gen corresponds to the final compiled obj name like 'a.out'
# the $(name) replaces with the value of the variable
# $@ is the name paint_gen

$(OFILES): $(HFILES)
 
# the above line means if the hfiles have been changed recompile the
# all the ofiles even if they haven't been changed
 

clean:
	rm $(OFILES) paint_gen
lint:
	lint $(CFILES)
