
template<typename T>
class BiquadFilter{
public:
    void init(float b0, float b1, float b2, float a1, float a2) {
        this->b0 = b0; this->b1 = b1; this->b2 = b2;
        this->a1 = a1; this->a2 = a2;

        x1 = x2 = y1 = y2 = T::Zero(); 
    }

    T apply(const T& x) {

        T y = b0*x + b1*x1 + b2*x2 - a1*y1 - a2*y2;

        x2 = x1; x1 = x;
        y2 = y1; y1 = y;
        return y;
    }
    void reset() { x1 = x2 = y1 = y2 = T::Zero(); }

private:
    float b0, b1, b2, a1, a2;
    T x1, x2, y1, y2; 
};
