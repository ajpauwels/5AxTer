#ifndef BUFFER_HPP
#define BUFFER_HPP

template<class T, int BUF_SIZE>
class Buffer {
private:
  T buf[BUF_SIZE];
  int nextPushIndex = 0;
  int nextPopIndex = 0;
  unsigned int size = 0;
  unsigned int playback_idx = 0;

  int incr(int idx, unsigned int num = 1) {
    int sum = idx + num;
    if (sum >= BUF_SIZE) {
      return sum - BUF_SIZE;
    } else {
      return sum;
    }
  }

  int decr(int idx, unsigned int num = 1) {
    int sum = idx - num;
    if (sum < 0) {
      return BUF_SIZE + sum;
    } else {
      return sum;
    }
  }

public:
  void seek(unsigned int idx) {
    if (idx < 0 || idx >= BUF_SIZE) return;
    playback_idx = idx;
  }

  void empty() {
    nextPushIndex = nextPopIndex = size = playback_idx = 0;
  }

  T next() {
    return buf[playback_idx++];
  }

  T prev() {
    playback_idx = decr(playback_idx, 2);
    T obj = buf[playback_idx];
    playback_idx = incr(playback_idx);

    return obj;
  }

  void restart() {
    playback_idx = nextPopIndex;
  }

  bool isFull() {
    return (size == BUF_SIZE);
  }

  bool isEmpty() {
    return (size == 0);
  }

  unsigned int getSize() {
    return size;
  }

  T peek() {
    return buf[nextPopIndex];
  }

  T pop() {
    int idx = nextPopIndex;
    nextPopIndex = incr(nextPopIndex);
    --size;

    if (nextPopIndex <= nextPushIndex) {
      if (playback_idx < nextPopIndex || playback_idx >= nextPushIndex) {
        playback_idx = nextPopIndex;
      }
    } else {
      if (playback_idx > nextPushIndex && playback_idx < nextPopIndex) {
        playback_idx = nextPopIndex;
      }
    }

    return buf[idx];
  }

  int push(T obj, bool overwrite = true) {
    if (size < BUF_SIZE || overwrite) {
      int idx = nextPushIndex;
      buf[idx] = obj;

      if (idx == nextPopIndex && isFull()) {
        nextPopIndex = incr(nextPopIndex);
      }
      nextPushIndex = incr(nextPushIndex);
      ++size;

      return idx;
    }

    return -1;
  }

};

#endif
