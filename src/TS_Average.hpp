#pragma once

#include "pzem_modbus.hpp"

/*
switch(meters->getState(id)->model) {
        case pzmodel_t::pzem004v3 : {
            pz004::rx_msg_prettyp(m);       // parse incoming message and print some nice info

            // or we can access struct data for the updated object (an example)
            auto *s = (const pz004::state*)meters->getState(id);
            Serial.printf("===\nPower alarm: %s\n", s->alarm ? "present" : "absent");
            Serial.printf("Power factor: %d\n", s->data.pf);
            Serial.printf("Current value: %f\n", s->data.asFloat(meter_t::cur));
            break;
        }
        case pzmodel_t::pzem003 : {
            pz003::rx_msg_prettyp(m);       // parse incoming message and print some nice info

            // or we can access struct data for the updated object
            auto *s = (const pz003::state*)meters->getState(id);
            Serial.printf("===\nPower high alarm: %s\n", s->alarmh ? "present" : "absent");
            Serial.printf("Power low alarm: %s\n", s->alarml ? "present" : "absent");
            Serial.printf("Energy: %d\n", s->data.energy);
            Serial.printf("Current value: %f\n", s->data.asFloat(meter_t::cur));
            break;
        }
        default:
            break;
    }
*/

template <class T>
class AveragingFunction {
   public:
	virtual ~AveragingFunction(){};

	virtual void   push(const T&) = 0;
	virtual T	   get()		  = 0;
	virtual void   reset()		  = 0;
	virtual size_t getCnt() const = 0;
};

template <class T>
class MeanAverage : public AveragingFunction<T> {
	unsigned v{0}, c{0}, p{0}, e{0}, f{0}, pf{0}, _cnt{0};

   public:
	void		   push(const T& m) override;
	T                  get() override;
	void		   reset() override;
	size_t		   getCnt() const override {
		return _cnt;
	};
};

template <class T>
void MeanAverage<T>::push(const T& m){
    
    v += m.voltage;
    c += m.current;
    p += m.power;
    e = m.energy;

   // if (std::is_same<T, int>::value) {
   if (std::is_same<T, pz004::metrics>::value) {
   // pz004::metrics
       f += m.freq;
       pf += m.pf;
   }
    ++_cnt;
}

template <class T>
T MeanAverage<T>::get(){
    T _m;
    _m.voltage = v / _cnt;
    _m.current = c / _cnt;
    _m.power = p / _cnt;
    _m.energy = e;
    if (std::is_same<T, pz004::metrics>::value) {
        _m.freq = f / _cnt;
        _m.pf = pf / _cnt;
    }
    return _m;
}

template <class T>
void MeanAverage<T>::reset(){
    if (std::is_same<T, pz004::metrics>::value) {
       v = c = p = e = f = pf = _cnt = 0;
    } else {
       v = c = p = e = _cnt = 0;    
    }
}



/*
namespace pz003 {   
class MeanAverage : public AveragingFunction<pz003::metrics> {
	unsigned v{0}, c{0}, p{0}, e{0}, _cnt{0};

   public:
	void		   push(const pz003::metrics& m) override;
	pz003::metrics get() override;
	void		   reset() override;
	size_t		   getCnt() const override {
		return _cnt;
	};
};



void MeanAverage::push(const pz003::metrics& m){
    v += m.voltage;
    c += m.current;
    p += m.power;
    e = m.energy;
    ++_cnt;
}

pz003::metrics MeanAverage::get(){
    pz003::metrics _m;
    _m.voltage = v / _cnt;
    _m.current = c / _cnt;
    _m.power = p / _cnt;
    _m.energy = e;
    return _m;
}

void MeanAverage::reset(){
    v = c = p = e  = _cnt = 0;
}

}

*/



// org
/*
template <class T>
class AveragingFunction {
   public:
	virtual ~AveragingFunction(){};

	virtual void   push(const T&) = 0;
	virtual T	   get()		  = 0;
	virtual void   reset()		  = 0;
	virtual size_t getCnt() const = 0;
};


class MeanAveragePZ004 : public AveragingFunction<pz004::metrics> {
	unsigned v{0}, c{0}, p{0}, e{0}, f{0}, pf{0}, _cnt{0};

   public:
	void		   push(const pz004::metrics& m) override;
	pz004::metrics get() override;
	void		   reset() override;
	size_t		   getCnt() const override {
		return _cnt;
	};
};


void MeanAveragePZ004::push(const pz004::metrics& m){
    v += m.voltage;
    c += m.current;
    p += m.power;
    e = m.energy;
    f += m.freq;
    pf += m.pf;
    ++_cnt;
}

pz004::metrics MeanAveragePZ004::get(){
    pz004::metrics _m;
    _m.voltage = v / _cnt;
    _m.current = c / _cnt;
    _m.power = p / _cnt;
    _m.energy = e;
    _m.freq = f / _cnt;
    _m.pf = pf / _cnt;
    return _m;
}

void MeanAveragePZ004::reset(){
    v = c = p = e = f = pf = _cnt = 0;
}
*/

