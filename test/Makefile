# Builds the host-side protocol self-test.
CFLAGS = -I../protocol -Wall -Wextra -std=c11
SRC = test_protocol.c ../protocol/crc.c ../protocol/gateway_protocol.c

test_protocol: $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $@

.PHONY: check clean
check: test_protocol
	./test_protocol

clean:
	rm -f test_protocol
