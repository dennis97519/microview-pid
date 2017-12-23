template <typename T>
class ContainedNum{
private:
  T val;
  T mini;
  T maxi;
  T range;
  bool wrap;
public:
  //do not use unsigned type
  T operator++(){
    T ret=val;
    operator+=(1);
    return ret;
  }
  T operator++(int){
    return operator+=(1);
  }
  T operator--(){
    T ret=val;
    operator-=(1);
    return ret;
  }
  T operator--(int){
    return operator-=(1);
  }
  T operator+=(T n){
    operator=(val+n);
    return val;
  }
  operator-=(T n){
    operator=(val-n);
    return val;
  }
  T operator=(T n){
   if(n>maxi){
      if(wrap) val=mini+(n-mini)%range;
      else val=maxi;
    }
   else if(n<mini){
      if(wrap){
        n+=range*((mini-n)/range+1);
        val=mini+(n-mini)%range;
      }
      else val=mini;
    }
    else val=n;
    return val;
  }
  operator T() const{
    return val;
  } 
  ContainedNum(T mini,T maxi,bool w=false){
    wrap=w;
    this->mini=mini;
    this->maxi=maxi;
    range=maxi-mini+1;
    if(range<=0)
    {
      range=-range;
      T temp=maxi;
      maxi=mini;
      mini=temp;
    }
    val=mini;
  }
};
