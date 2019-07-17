
#ifndef GeneratorPQ__Dyn_h
#define GeneratorPQ__Dyn_h


#include "DYNModelModelicaDyn.h"
#include "DYNModelManagerCommon.h"
#include "PARParametersSet.h"
#include "PARParametersSetFactory.h"
#include "PARParameter.h"
#include "DYNSubModel.h"
#include "DYNVariableForModel.h"
#include "DYNParameter.h"
#ifdef _ADEPT_
#include "adept.h"
#endif

namespace DYN {

  class ModelGeneratorPQ_Dyn : public ModelModelicaDyn
  {
    public:
    ModelGeneratorPQ_Dyn() {dataStructIsInitialized_ = false;}
    ~ModelGeneratorPQ_Dyn() {if (dataStructIsInitialized_) deInitializeDataStruc();}


    public:
    void initData(DYNDATA * d);
    void initRpar();
    void setFomc(double * f);
    void setGomc(state_g * g);
    modeChangeType_t evalMode(const double & t) const;
    void setZomc();
    void setOomc();
    void setY0omc();
    void setYType_omc(propertyContinuousVar_t* yType);
    void setFType_omc(propertyF_t* fType);
    boost::shared_ptr<parameters::ParametersSet> setSharedParametersDefaultValues();
    void setParameters( boost::shared_ptr<parameters::ParametersSet> params );
    void defineVariables(std::vector< boost::shared_ptr<Variable> >& variables);
    void defineParameters(std::vector<ParameterModeler>& parameters);
    void defineElements(std::vector<Element> &elements, std::map<std::string, int>& mapElement);

#ifdef _ADEPT_
    void evalFAdept(const std::vector<adept::adouble> &y, const std::vector<adept::adouble> &yp, std::vector<adept::adouble> &F);
#endif

    void checkDataCoherence ();
    void setFequations (std::map<int,std::string>& fEquationIndex);
    void setGequations (std::map<int,std::string>& gEquationIndex);

    inline void setModelType(std::string modelType) { modelType_ = modelType; }
    inline ModelManager * getModelManager() const { return modelManager_; }
    inline void setModelManager (ModelManager * model) { modelManager_ = model; }
    void checkSum(std::string & checkSum) { checkSum = std::string("dceade702237169759a1ea66ac59dfe7"); }

    private:
    DYNDATA * data;
    ModelManager * modelManager_;
    bool dataStructIsInitialized_;
    std::string modelType_;

    private:
    std::string modelType() const { return modelType_; }
    inline void setData(DYNDATA * d){ data = d; }
    void setupDataStruc();
    void initializeDataStruc();
    void deInitializeDataStruc();

    private:
   //External Calls
     typedef struct Complex_s {
       modelica_real _im;
       modelica_real _re;
     } Complex;
     typedef base_array_t Complex_array;
     typedef Complex Dynawo_Types_AC_ApparentPower;
     typedef base_array_t Dynawo_Types_AC_ApparentPower_array;
     typedef Complex Dynawo_Types_AC_ApparentPower$generator$SGenPu;
     typedef base_array_t Dynawo_Types_AC_ApparentPower$generator$SGenPu_array;
     typedef Complex Dynawo_Types_AC_Current;
     typedef base_array_t Dynawo_Types_AC_Current_array;
     typedef Complex Dynawo_Types_AC_Current$generator$terminal$i;
     typedef base_array_t Dynawo_Types_AC_Current$generator$terminal$i_array;
     typedef Complex Dynawo_Types_AC_Voltage;
     typedef base_array_t Dynawo_Types_AC_Voltage_array;
     typedef Complex Dynawo_Types_AC_Voltage$generator$terminal$V;
     typedef base_array_t Dynawo_Types_AC_Voltage$generator$terminal$V_array;
     Complex omc_Complex ( modelica_real omc_re, modelica_real omc_im);
     Complex omc_Complex__omcQuot_2B( Complex _c1, Complex _c2);
     Complex omc_Complex__omcQuot_2A_multiply( Complex _c1, Complex _c2);
     Complex omc_Complex__omcQuot_2A_scalarProduct( Complex_array _c1, Complex_array _c2);
     Complex omc_Complex__omcQuot_2D_negate( Complex _c1);
     Complex omc_Complex__omcQuot_636F6E7374727563746F72_fromReal( modelica_real _re, modelica_real _im);
     void omc_Dynawo_NonElectrical_Logs_Timeline_logEvent1( modelica_integer _key);
     void boxptr_Dynawo_NonElectrical_Logs_Timeline_logEvent1( modelica_metatype _key);
     Dynawo_Types_AC_ApparentPower omc_Dynawo_Types_AC_ApparentPower ( modelica_real omc_re, modelica_real omc_im);
     Dynawo_Types_AC_ApparentPower$generator$SGenPu omc_Dynawo_Types_AC_ApparentPower$generator$SGenPu ( modelica_real omc_re, modelica_real omc_im);
     Dynawo_Types_AC_Current omc_Dynawo_Types_AC_Current ( modelica_real omc_re, modelica_real omc_im);
     Dynawo_Types_AC_Current$generator$terminal$i omc_Dynawo_Types_AC_Current$generator$terminal$i ( modelica_real omc_re, modelica_real omc_im);
     Dynawo_Types_AC_Voltage omc_Dynawo_Types_AC_Voltage ( modelica_real omc_re, modelica_real omc_im);
     Dynawo_Types_AC_Voltage$generator$terminal$V omc_Dynawo_Types_AC_Voltage$generator$terminal$V ( modelica_real omc_re, modelica_real omc_im);
     modelica_real omc_Modelica_ComplexMath__omcQuot_616273( Complex _c);
     modelica_metatype boxptr_Modelica_ComplexMath__omcQuot_616273( modelica_metatype _c);
     Complex omc_Modelica_ComplexMath_conj( Complex _c1);
     modelica_metatype boxptr_Modelica_ComplexMath_conj( modelica_metatype _c1);

      // Non-internal parameters 
      double generator_AlphaPu_;
      double generator_PGen0Pu_;
      double generator_PMaxPu_;
      double generator_PMinPu_;
      double generator_QGen0Pu_;
      double generator_QMaxPu_;
      double generator_QMinPu_;
      double generator_U0Pu_;
      double generator_UMaxPu_;
      double generator_UMinPu_;
      double generator_i0Pu_im_;
      double generator_i0Pu_re_;
      double generator_u0Pu_im_;
      double generator_u0Pu_re_;
      int generator_NbSwitchOffSignals_;
      int generator_State0_;

  };
}//end namespace DYN

#endif

