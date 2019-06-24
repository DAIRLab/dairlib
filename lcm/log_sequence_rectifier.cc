#include <string.h>
#include <inttypes.h>
#include <lcm/lcm.h>
#include <iostream>
#include <list>

/**
  This is a simple program to fix any LCM messages that may be out of sequence
  in a log file. The usage is:

    log_sequence_rectifier <file_in> <file_out>

  The program uses a max-length buffer, that must be large enough so that out of
  sequence messages must appear at most kBufferLength-messages apart. An error
  message will be generated if this assumption fails.
*/

constexpr int kBufferLength = 20;

bool event_sorter(lcm_eventlog_event_t* const& lhs,
    lcm_eventlog_event_t* const& rhs) {
  return lhs->timestamp < rhs->timestamp;
}

int main(int argc, char **argv) {
  int N = 0;
  std::list<lcm_eventlog_event_t*> buffer;

  lcm_eventlog_t *log, *log_out;
  if (argc < 3) {
    fprintf(stderr, "usage: log_sequence_rectifier <file_in> <file_out>\n");
    return 1;
  }

  log = lcm_eventlog_create(argv[1], "r");

  if (!log) {
    fprintf(stderr, "couldn't open input log file\n");
    return 1;
  }

  log_out = lcm_eventlog_create(argv[2], "w");
  if (!log_out) {
    fprintf(stderr, "couldn't open output log file\n");
    return 1;
  }

  int64_t last_timestamp = 0;
  int64_t last_write_timestamp = 0;
  while (1) {
    lcm_eventlog_event_t *event = lcm_eventlog_read_next_event(log);


    if (!event) {
      break;
    }

    // Copy the event and add to the buffer
    lcm_eventlog_event_t* e = new lcm_eventlog_event_t;
    e->eventnum = event->eventnum;
    e->timestamp = event->timestamp;
    e->channellen = event->channellen;
    e->datalen = event->datalen;
    e->channel = strdup(event->channel);
    e->data = malloc(e->datalen);
    memcpy(e->data, event->data, e->datalen);
    buffer.push_back(e);
    N++;

    // If buffer is long enough, sort and write oldest message
    if (N == kBufferLength) {
      buffer.sort(event_sorter);
      lcm_eventlog_write_event(log_out, buffer.front());

      int64_t timestamp = buffer.front()->timestamp;
      if (timestamp < last_write_timestamp) {
        std::cout <<
            "ERROR: buffer size too small to capture out-of-order message!"
            << std::endl;
      }
      last_write_timestamp = timestamp;



      // Free and remove message
      free(buffer.front()->channel);
      free(buffer.front()->data);
      delete buffer.front();
      buffer.pop_front();
      N--;
    }

    if (event->timestamp < last_timestamp) {
      std::cout << "Found a message out of order." << std::endl;
    }

    last_timestamp = event->timestamp;

    lcm_eventlog_free_event(event);
  }

  // Write remaining messages
  buffer.sort(event_sorter);
  while (N > 0) {
    lcm_eventlog_write_event(log_out, buffer.front());

      int64_t timestamp = buffer.front()->timestamp;
      if (timestamp < last_write_timestamp) {
        std::cout <<
            "ERROR: buffer size too small to capture out-of-order message!"
            << std::endl;
      }
      last_write_timestamp = timestamp;


    // Free and remove message
    free(buffer.front()->channel);
    free(buffer.front()->data);
    delete buffer.front();
    buffer.pop_front();
    N--;
  }

  lcm_eventlog_destroy(log);
  return 0;
}
