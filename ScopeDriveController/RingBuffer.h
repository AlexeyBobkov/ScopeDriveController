/*
 * RingBuffer.h
 *
 * Created: 2/22/2019 9:59:04 AM
 *  Author: Alexey
 */


#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

template<class T, int Size>
class RingBuffer
{
    inline static int next(int pos) {return (pos == Size - 1) ? 0 : (pos + 1);}

public:
    RingBuffer() : from_(0), to_(0), cnt_(0), overflowedCnt_(0) {}

    bool push_back(const T &x)
    {
        buffer_[to_] = x;
        to_ = next(to_);
        if(cnt_ < Size)
        {
            ++cnt_;
            return true;
        }
        else
        {
            from_ = to_;
            ++overflowedCnt_;
            return false;
        }
    }

    T&          front()         {return buffer_[from_];}
    const T&    front() const   {return buffer_[from_];}

    void pop_front()
    {
        if(cnt_ > 0)
        {
            --cnt_;
            from_ = next(from_);
        }
    }

    int count() const           {return cnt_;}
    int overflowedCount() const {return overflowedCnt_;}
    void resetOverflowedCnt()   {overflowedCnt_ = 0;}
    void clear()                {from_ = to_ = cnt_ = overflowedCnt_ = 0;}

private:
    T buffer_[Size];
    int from_, to_, cnt_, overflowedCnt_;
};



#endif /* RINGBUFFER_H_ */