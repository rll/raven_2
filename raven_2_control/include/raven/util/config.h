#ifndef ROSX_CONFIG
#define ROSX_CONFIG

#ifndef ROSX_STRINGIFY
#define ROSX_STRINGIFY(X) ROSX_STRINGIFY_HELPER(X)
#define ROSX_STRINGIFY_HELPER(X) #X
#endif


/* Modified from code I got from John Schulman */

#include <vector>
#include <string>
#include <iostream>
#include <sstream>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

namespace po = boost::program_options;

namespace rosx {

struct ConfigGroup {
	std::string name;
	po::options_description options;

	ConfigGroup(const std::string& the_name = "") : name(the_name), options(name) {}


#define ConfigGroup_option(field, type, ...) addOption<type>(rosx::ConfigGroup::underscore_to_hyphen(ROSX_STRINGIFY(field)),&(this->field), ##__VA_ARGS__)
#define ConfigGroup_options(field, other_names, type, ...) addOption<type>(rosx::ConfigGroup::underscore_to_hyphen(ROSX_STRINGIFY(field)),other_names, &(this->field), ##__VA_ARGS__)
#define ConfigGroup_optionWithHelp(field, type, help, ...) addOptionWithHelp<type>(rosx::ConfigGroup::underscore_to_hyphen(ROSX_STRINGIFY(field)),&(this->field), help, ##__VA_ARGS__)
#define ConfigGroup_optionsWithHelp(field, other_names, type, help, ...) addOptionWithHelp<type>(rosx::ConfigGroup::underscore_to_hyphen(ROSX_STRINGIFY(field)),other_names, &(this->field), help, ##__VA_ARGS__)

#define ConfigGroup_flag(field, ...) addFlag(rosx::ConfigGroup::underscore_to_hyphen(ROSX_STRINGIFY(field)),&(this->field), ##__VA_ARGS__)
#define ConfigGroup_flags(field, other_names, ...) addFlag(rosx::ConfigGroup::underscore_to_hyphen(ROSX_STRINGIFY(field)),other_names, &(this->field), ##__VA_ARGS__)
#define ConfigGroup_flagWithHelp(field, help, ...) addFlagWithHelp(rosx::ConfigGroup::underscore_to_hyphen(ROSX_STRINGIFY(field)),&(this->field), help, ##__VA_ARGS__)
#define ConfigGroup_flagsWithHelp(field, other_names, help, ...) addFlagWithHelp(rosx::ConfigGroup::underscore_to_hyphen(ROSX_STRINGIFY(field)),other_names, &(this->field), help, ##__VA_ARGS__)


	template<typename T>
	void addOption(const std::string& name,T* value,T default_value=T()) {
		options.add_options()(name.c_str(), po::value(value)->default_value(default_value), "");
	}

	template<typename T>
	void addOption(const std::string& name,const std::string& other_names, T* value,T default_value=T()) {
		std::string names = name + "," + other_names;
		options.add_options()(names.c_str(), po::value(value)->default_value(default_value), "");
	}

	template<typename T>
	void addOptionWithHelp(const std::string& name,T*value,const std::string& help,T default_value=T()) {
		options.add_options()(name.c_str(), po::value(value)->default_value(default_value), help.c_str());
	}

	template<typename T>
	void addOptionWithHelp(const std::string& name,const std::string& other_names,T*value,const std::string& help,T default_value=T()) {
		std::string names = name + "," + other_names;
		options.add_options()(names.c_str(), po::value(value)->default_value(default_value), help.c_str());
	}

	void addFlag(const std::string& name,bool* value,bool flag_value=true) {
		options.add_options()(name.c_str(), po::value(value)->zero_tokens()->default_value(!flag_value)->implicit_value(flag_value), "");
	}

	void addFlag(const std::string& name,const std::string& other_names, bool* value,bool flag_value=true) {
		std::string names = name + "," + other_names;
		options.add_options()(names.c_str(), po::value(value)->zero_tokens()->default_value(!flag_value)->implicit_value(flag_value), "");
	}

	void addFlagWithHelp(const std::string& name,bool*value,const std::string& help,bool flag_value=true) {
		options.add_options()(name.c_str(), po::value(value)->zero_tokens()->default_value(!flag_value)->implicit_value(flag_value), help.c_str());
	}

	void addFlagWithHelp(const std::string& name,const std::string& other_names,bool*value,const std::string& help,bool flag_value=true) {
		std::string names = name + "," + other_names;
		options.add_options()(names.c_str(), po::value(value)->zero_tokens()->default_value(!flag_value)->implicit_value(flag_value), help.c_str());
	}

	static inline std::string underscore_to_hyphen(std::string val) {
		for (size_t i=0;i<val.size();i++) {
			if (val[i] == '_') {
				val[i] = '-';
			}
		}
		return val;
	}
};

class Parser {
	std::vector<ConfigGroup*> m_configs;
	po::options_description m_options;
	po::positional_options_description m_positional_args;
	po::variables_map m_vm;
	bool m_read;
public:
	Parser() : m_read(false) { }
	
	void addGroup(ConfigGroup& config) {
		m_configs.push_back(&config);
	}
	void addGroup(ConfigGroup* config) {
		m_configs.push_back(config);
	}
	template<typename T>
	void addOption(const std::string& name,T default_value=T()) {
		m_options.add_options()(name.c_str(), po::value<T>()->default_value(default_value));
	}
	void addFlag(const std::string& name,bool flag_value=true) {
		m_options.add_options()(name.c_str(), po::value<bool>()->zero_tokens()->default_value(!flag_value)->implicit_value(flag_value));
	}
	template<typename T>
	void addArg(const std::string& name,int max_count=1) {
		m_options.add_options()(name.c_str(), po::value<T>());
		m_positional_args.add(name.c_str(),max_count);
	}

	void read(int argc, char* argv[]) {
		// create boost options_description based on variables, parser
		po::options_description od;
		od.add_options()("help,h", "produce help message");
		od.add(m_options);
		BOOST_FOREACH(ConfigGroup* config, m_configs) {
			od.add(config->options);
		}
		po::store(
				po::command_line_parser(argc, argv).options(od).positional(m_positional_args).run(),
				m_vm);
		if (m_vm.count("help")) {
			std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
			std::cout << od << std::endl;
			exit(0);
		}
		po::notify(m_vm);
		m_read = true;
	}
	
	bool hasOption(const std::string& name) {
		return m_vm.count(name);
	}
	template <typename T>
	T getOption(const std::string& name) {
		return m_vm[name].as< T >();
	}
	
	bool getFlag(const std::string& name) {
		return m_vm[name].as<bool>();
	}

	bool hasArg(const std::string& name) {
		return hasOption(name);
	}
	template <typename T>
	T getArg(const std::string& name) {
		return getOption<T>(name);
	}
	template <typename T>
	T getArg(int pos) {
		return m_vm[m_positional_args.name_for_position(pos)].as< T >();
	}
};

template<typename T>
std::vector<T> parseVector(const std::string& str) {
	std::vector<T> v;
	std::stringstream ss(str);
	while (!ss.eof()) {
		T value;
		ss >> value;
		v.push_back(value);
		if (ss.fail()) {
			v.clear();
			std::cerr << "Could not parse " << value << " from " << str << " into " << typeid(T) << std::endl;
			break;
		}
	}
	return v;
}

} //namespace rosx

struct Config : public rosx::ConfigGroup {
	bool disable_gold_grasp2;
	bool use_new_cable_coupling;
	bool use_new_kinematics;

	Config() : rosx::ConfigGroup() {
		ConfigGroup_flag(disable_gold_grasp2);
		ConfigGroup_flag(use_new_cable_coupling);
		ConfigGroup_flag(use_new_kinematics);
//		ConfigGroup_option(param1,float);
//		ConfigGroup_option(param2_has_default,std::string,"thedefault");
//		ConfigGroup_options(param3,"v,param-number-three",int);
	}

	static Config Options;
};
#define RavenConfig Config::Options

#endif
