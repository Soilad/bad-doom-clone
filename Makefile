all :
	clang -O2 main.c -o game -fsanitize=address -pg -Wall -lSDL2 -lSDL2_image
