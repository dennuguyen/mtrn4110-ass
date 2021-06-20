// TODO: Implement this.
// https://gitlab.cse.unsw.edu.au/COMP2511/20T3/m17a-and_i_oop/-/blob/master/src/unsw/gloriaromanus/util/PubSub.java

#ifndef PUB_SUB_HPP_
#define PUB_SUB_HPP_

#include <map>
#include <vector>

class PubSub {
   public:
    PubSub(){};

    auto getInstance() -> PubSub {}

    template <typename T>
    auto publish(std::string topic, T t) -> void {}

    template <typename T>
    auto listen(std::string topic, T t) -> void {}

    auto subscribe(PubSub subscriber, std::string topic) -> void {}

    auto unsubscribe(PubSub subscriber, std::string) -> void {}

   private:
    std::map<std::string, std::vector<PubSub>> subscribers_;
};

#endif  // PUB_SUB_HPP_