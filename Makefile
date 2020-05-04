CC=gcc
CFLAGS=-I. -l bcm2835 -lm
DEPS = ili9341_spi.h glcdfont.h
OBJ = ili9341_spi.o weather_graph.o

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

weather_graph: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)

clean:
	rm -rf $(OBJ)
