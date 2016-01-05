class NewModule : public DerWeg::KogmoThread {
  // some private attributes:
  ...
public:
  // default constructor, if required
  NewModule () { ... }
  // destructor, if required
  ~NewModule () { ... }
  // init-function, if required
  void init (const ConfigReader& cfg) { ... }
  // deinit-function, if required
  void deinit () { ... }
  // execute-function, mandatory
  void execute () { .... }
};