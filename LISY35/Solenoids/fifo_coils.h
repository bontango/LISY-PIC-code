/* 
 * File:   fifo.h
 * Author: th
 *
 * Created on 23. Oktober 2015, 18:09
 */

#ifndef FIFO_H
#define	FIFO_H

// FIFO Handling */
#define BUFFER_FAIL     0
#define BUFFER_SUCCESS  1

#define BUFFER_SIZE 128

struct Buffer {
  uint8_t data[BUFFER_SIZE];
  uint8_t read; // zeigt auf das Feld mit dem ältesten Inhalt
  uint8_t write; // zeigt immer auf leeres Feld
} buffer;

//
// Stellt 1 Byte in den Ringbuffer
//
// Returns:
//     BUFFER_FAIL       der Ringbuffer ist voll. Es kann kein weiteres Byte gespeichert werden
//     BUFFER_SUCCESS    das Byte wurde gespeichert
//
uint8_t BufferIn(uint8_t byte)
{

  if ( ( buffer.write + 1 == buffer.read ) ||
       ( buffer.read == 0 && buffer.write + 1 == BUFFER_SIZE ) )
    return BUFFER_FAIL; // voll

  buffer.data[buffer.write] = byte;

  buffer.write++;
  if (buffer.write >= BUFFER_SIZE)
    buffer.write = 0;

  return BUFFER_SUCCESS;
}
//
// Holt 1 Byte aus dem Ringbuffer, sofern mindestens eines abholbereit ist
//
// Returns:
//     BUFFER_FAIL       der Ringbuffer ist leer. Es kann kein Byte geliefert werden.
//     BUFFER_SUCCESS    1 Byte wurde geliefert
//
uint8_t BufferOut(uint8_t *pByte)
{
  if (buffer.read == buffer.write)
    return BUFFER_FAIL;

  *pByte = buffer.data[buffer.read];

  buffer.read++;
  if (buffer.read >= BUFFER_SIZE)
    buffer.read = 0;

  return BUFFER_SUCCESS;
}

//
// Fragt den Status des Ringbuffers ab
//
// Returns:
//     TRUE (1)       mindestens ein Byte im Ringbuffer
//     FALSE (0)      Ringbuffer empty
//
uint8_t BufferStatus(void) {
    if (buffer.read == buffer.write)
      return 0;
    else
        return 1;
}
        
        

#endif	/* FIFO_H */

