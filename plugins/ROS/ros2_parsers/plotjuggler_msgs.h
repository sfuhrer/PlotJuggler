#pragma once

#include "pj_msgs/msg/dictionary.hpp"
#include "pj_msgs/msg/data_points.hpp"
#include "ros2_parser.h"

using Ros2Introspection::BuiltinMessageParser;

static std::unordered_map<unsigned, std::vector<std::string>> _pj_msgs_dictionaries;

class PlotJugglerDictionaryParser: public BuiltinMessageParser<pj_msgs::msg::Dictionary>
{
public:

    PlotJugglerDictionaryParser(const std::string& topic_name):
      BuiltinMessageParser<pj_msgs::msg::Dictionary>(topic_name)
    { }

    virtual void setMaxArrayPolicy(Ros2Introspection::MaxArrayPolicy, size_t) {}

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const pj_msgs::msg::Dictionary& msg,
                          double timestamp) override
    {
      _pj_msgs_dictionaries[ msg.dictionary_uuid ] = msg.names;
    }
};

//------------------------------------
class PlotJugglerDataPointsParser: public BuiltinMessageParser<pj_msgs::msg::DataPoints>
{
public:

    PlotJugglerDataPointsParser(const std::string& topic_name):
      BuiltinMessageParser<pj_msgs::msg::DataPoints>(topic_name)
    {
      _prefix = topic_name + "/";
    }

    virtual void setMaxArrayPolicy(Ros2Introspection::MaxArrayPolicy, size_t) {}

    void parseMessageImpl(PlotDataMapRef& plot_data,
                          const pj_msgs::msg::DataPoints& msg,
                          double timestamp) override
    {
      auto it = _pj_msgs_dictionaries.find( msg.dictionary_uuid );
      if( it == _pj_msgs_dictionaries.end() )
      {
        const auto& names = it->second;
        for( const auto& sample: msg.samples)
        {
          auto& series = getSeries(plot_data, _prefix + std::to_string(sample.name_index));
          series.pushBack( {sample.stamp, sample.value} );
        }
      }
      else{
        const auto& names = it->second;
        for( const auto& sample: msg.samples)
        {
          auto& series = getSeries(plot_data, _prefix + names[sample.name_index]);
          series.pushBack( {sample.stamp, sample.value} );
        }
      }
    }
private:
    std::string _prefix;
};