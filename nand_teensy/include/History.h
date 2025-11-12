template <typename T, size_t Count>
class History
{
public:
  History() : length(0) {}

  void push(T t)
  {
    if (length >= Count)
    {
      // History is full, discard an element
      for (size_t i = 0; i < Count - 1; ++i)
      {
        data[i] = std::move(data[i + 1]);
      }
      data[Count - 1] = std::move(t);
    }
    else
    {
      data[length++] = std::move(t);
    }
  }

  T max()
  {
    if (length == 0)
    {
      return {};
    }
    else
    {
      T m = data[0];
      for (size_t i = 1; i < length; ++i)
      {
        if (m < data[i])
        {
          m = data[i];
        }
      }
      return m;
    }
  }

  double avg()
  {
    double a = 0.0;
    for (size_t i = 0; i < length; ++i)
    {
      a += data[i];
    }
    return a / length;
  }

private:
  T data[Count];
  size_t length;
};
