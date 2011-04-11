#include "script_engine/engine_v8.hpp"
#include "script_engine/engine_module.hpp"

using namespace script_engine;

namespace {
	v8::Handle<v8::Value> se_log(const v8::Arguments& args)
	{
		v8::String::Utf8Value v(args[0]);
		const char* val = *v;
		ROS_INFO("se_log: %s", val);
		return v8::True();
	}
}

engine_v8::engine_v8():
//	global_(v8::ObjectTemplate::New()),
//	context_(v8::Context::New(NULL, global_)),
	module_loader_(new module_loader_t("script_engine",
		"script_engine::engine_module"))
{
	ros::NodeHandle n;
	srv_eval_script_ = n.advertiseService("eval_script",
		&engine_v8::eval_srv, this);
	srv_compile_script_ = n.advertiseService("compile_script",
		&engine_v8::compile_srv, this);
	srv_run_script_ = n.advertiseService("run_script",
		&engine_v8::run_srv, this);

	global_ = v8::ObjectTemplate::New();
		
	// Log function.
	global_->Set(v8::String::New("se_log"),
		v8::FunctionTemplate::New(se_log));

	// Load plugins.
	std::vector<std::string> classes = module_loader_->getDeclaredClasses();
	std::vector<std::string>::const_iterator i;
	for (i = classes.begin(); i != classes.end(); ++i)
	{
		try 
		{
			engine_module* m = module_loader_->createClassInstance(*i);
			m->init(global_);
			modules_list_.push_back(m);
		}
		catch (pluginlib::LibraryLoadException e)
		{
			ROS_ERROR("Cannot load script_engine plugin, exception: \n %s", 
				e.what());
		}
	}

	context_ = v8::Context::New(NULL, global_);
	
}

engine_v8::~engine_v8()
{
	modules_list_t::iterator i;
	for (i = modules_list_.begin(); i != modules_list_.end(); ++i)
		delete *i;
}

bool engine_v8::eval(const std::string& src, std::string& result)
{
	ROS_INFO("eval(\"%s\") ...", src.c_str());
	using namespace v8;
	Context::Scope context_scope(context_);
	Handle<String> str = String::New(src.c_str());
	Handle<Script> script = Script::Compile(str);

	return run_script(script, result);
}

bool engine_v8::eval_srv(EvalScript::Request& req, EvalScript::Response& res)
{
	eval(req.source, res.result);

	return true;
}

bool engine_v8::compile(const std::string& name, const std::string& src)
{
	using namespace v8;
	Context::Scope context_scope(context_);
	scripts_map_[name] = Script::Compile(String::New(src.c_str()));

	return true;
}

bool engine_v8::compile_srv(CompileScript::Request& req,
	CompileScript::Response& res)
{
	return compile(req.name, req.source);
}

bool engine_v8::run(const std::string& name, std::string& result)
{
	v8::Context::Scope context_scope(context_);
	return run_script(scripts_map_[name], result);
	//scripts_map_[name]->Run();

	return true;
}

bool engine_v8::run_srv(RunScript::Request& req, RunScript::Response& res)
{
	return run(req.name, res.result);
}

bool engine_v8::run_script(const v8::Handle<v8::Script>& s, std::string& result)
{
	using namespace v8;

	TryCatch tc;
	Handle<Value> v = s->Run();
	if (v.IsEmpty())
	{
		Handle<Value> exception = tc.Exception();
		String::AsciiValue e_str(exception);
		ROS_ERROR("Exception: %s", *e_str);
		return false;
	}
	
	v8::String::Utf8Value rs(v);
	result = std::string(*rs, rs.length());

	return true;

}