// -------------------------------------------------------------- -*- C++ -*-
// File: ./include/ilopl/ilooplmodel.h
// --------------------------------------------------------------------------
// Licensed Materials - Property of IBM
//
// 5725-A06 5725-A29 5724-Y48 5724-Y49 5724-Y54 5724-Y55 
// Copyright IBM Corp. 1998, 2013
//
// US Government Users Restricted Rights - Use, duplication or
// disclosure restricted by GSA ADP Schedule Contract with
// IBM Corp.
// ---------------------------------------------------------------------------

#ifndef __OPL_ilooplmodelH
#define __OPL_ilooplmodelH

#ifdef _WIN32
#pragma pack(push, 8)
#endif


#include <ilopl/ilooplsettings.h>
#include <ilopl/ilooplmodeldefinition.h>
#include <ilopl/ilooplelementdefinition.h>
#include <ilopl/iloopldatasource.h>
//#include <ilopl/ilooplprofiler.h>

#include <ilopl/ilooplmodeli.h>

#include <ilcplex/ilocplex.h>

class IloCP;



extern const char* IloOplGetVersion();


extern IloBool IloOplIsRunOnServer();


extern void IloOplSetRunOnServer();



extern int IloOplRunMain(int argc, const char* argv[]);



void IloOplPrepareAll();


void IloOplEndAll();


void IloOplFormatExportFile(const char* fromfilename, const char* tofilename);

IloBool IloOplRemoveVarExportFile(const char* fromfilename, const char* tofilename, const char* name);



extern void IloOplSetDebug(IloInt level);


const char* IloOplGetLimitationMessage(IloInt mode);

class IloOplSolutionGetter;
class IloOplResultPublisher;
class IloOplScriptExpression;
class IloOplLabelCallback;
class IloOplCustomDataHandler;
class IloOplDecisionExprCallback;
class IloOplDecisionExprSolutionCallback;
class IloOplConflictIterator;
class IloOplRelaxationIterator;


class IloOplRunseedProgressI;
class IloOplDisposeListenerI;


class IloOplRunseedProgressI : public IloEnvObjectI {
public:
	IloOplRunseedProgressI(IloEnv env) : IloEnvObjectI(env.getImpl()) {}
	virtual ~IloOplRunseedProgressI() {}
	
	virtual void run(IloInt nb) = 0;
};


class IloOplRunseedProgress {
	IloOplRunseedProgressI* _impl;
public:
	
	IloOplRunseedProgress(IloOplRunseedProgressI* l) : _impl(l) {}
	
	void run(IloInt nb);
	
	void end() { delete _impl; _impl = 0; }
	
	IloOplRunseedProgressI* getImpl() { return _impl; }
	
	void setImpl(IloOplRunseedProgressI* impl) { _impl = impl; }
};



class IloOplDisposeListener {
	IloOplDisposeListenerI* _impl;
public:
	
	IloOplDisposeListener(IloOplDisposeListenerI* l);
	
	void end();
	
	void dispose(IloOplModel opl);
	
	IloOplDisposeListenerI* getImpl() { return _impl; }
	
	void setImpl(IloOplDisposeListenerI* impl) { _impl = impl; }
};


class IloOplPostProcessListenerI;

class IloOplPostProcessListener{
	IloOplPostProcessListenerI* _impl;
public:
	
	IloOplPostProcessListener(IloOplPostProcessListenerI* l);
	
	void startPostProcess();
	
	void endPostProcess();
	
	IloOplPostProcessListenerI* getImpl(){ return _impl; }
	
	void setImpl(IloOplPostProcessListenerI* impl){ _impl = impl; }
};




class ILOOPL_EXPORTED IloOplModel {
    HANDLE_DECL_OPL(IloOplModel)
public:
    
    IloOplModel(IloOplModelDefinition definition, IloCplex cplex):_impl(0) {
        _impl = new (definition.getEnv()) IloOplCplexModelI(definition.getEnv().getImpl(),definition.impl(),*cplex.getImpl());
    }
    
    IloOplModel(IloEnv env, IloOplModelDefinition definition, IloCplex cplex):_impl(0) {
		if (env.getImpl() != definition.getEnv().getImpl()) throw IloEnvironmentMismatch();
        _impl = new (env) IloOplCplexModelI(env.getImpl(),definition.impl(),*cplex.getImpl());
    }

    
    IloOplModel(IloOplSettings settings, IloOplModelDefinition definition, IloCplex cplex):_impl(0) {
        _impl = new (definition.getEnv()) IloOplCplexModelI(definition.getEnv().getImpl(),definition.impl(),*cplex.getImpl(),settings.impl());
    }

    
    IloOplModel(IloEnv env, IloOplSettings settings, IloOplModelDefinition definition, IloCplex cplex):_impl(0) {
		if (env.getImpl() != definition.getEnv().getImpl()) throw IloEnvironmentMismatch();
        _impl = new (env) IloOplCplexModelI(env.getImpl(),definition.impl(),*cplex.getImpl(),settings.impl());
    }

    
    IloOplModel(IloOplModelDefinition definition, IloCP cp);

    
    IloOplModel(IloEnv env, IloOplModelDefinition definition, IloCP cp);

    
    IloOplModel(IloOplSettings settings, IloOplModelDefinition definition, IloCP cp);

    
    IloOplModel(IloEnv env, IloOplSettings settings, IloOplModelDefinition definition, IloCP cp);


    
    const char* getName() const {
        return impl().getName();
    }

    
    const char* getSourceName() const {
        return impl().getSourceName();
    }

		
	void applyOpsSettings(const char* baseDir, const char* opsFile) {
		return impl().applyOpsSettings(baseDir, opsFile);
	}

    
    const char* resolvePath(const char* name) const {
        return impl().resolvePath(name);
    }

    
    std::istream* resolveStream(const char* name) const {
        return impl().resolveStream(name);
    }

    
    IloInt printRelaxation(std::ostream& os) const {
		    return impl().printRelaxation(os);
    }

    
    IloInt printConflict(std::ostream& os) const {
		    return impl().printConflict(os);
    }

    
    IloOplSettings getSettings() const {
        return &impl().getSettings();
    }

    
    void setSettings(IloOplSettings settings) {
        return impl().setSettings(settings.impl());
    }

    
    IloOplModelDefinition getModelDefinition() const {
        return &impl().getModelDefinition();
    }

    
    void addDataSource(IloOplDataSource source) {
        impl().addDataSource(source.impl());
    }


    
    void addSettings(IloOplSettings settings) {
        impl().addSettings(settings.impl());
    }

    
    void addDataSources(IloOplDataSourceArray sources) {
        for(int i=0; i<sources.getSize(); i++) {
            impl().addDataSource(sources[i].impl());
        }
	}

	
	void runSeed(IloInt n) {
		impl().runSeed(n);
	}

	
	void setDelayExtraction();
	
	void unsetDelayExtraction(); 


	
	IloBool solveAlgorithm();

	
	void registerPostProcessListener(IloOplPostProcessListener d);
	
	void unregisterPostProcessListener(IloOplPostProcessListener d);

	
	void registerRunseedProgressListener(IloOplRunseedProgress d);
	
	void unregisterRunseedProgressListener(IloOplRunseedProgress d);

	
	IloBool internalSolve(); 	
	
    IloOplElementIterator makeElementIterator() {
        return impl().makeElementIterator();
    }

    
    IloOplElementIterator makeElementIterator(IloBool ownElements) {
        return impl().makeElementIterator(ownElements);
    }

    
    IloOplElement getElement(const char* name) {
        return &impl().getElement(name);
    }

   
    IloBool hasElement(const char* name) const {
        return impl().hasElement(name);
    }

    
    IloStringArray getElementNamesInPostProcessing() const {
        return impl().getElementNamesInPostProcessing();
    }

    
    void setZeroSolutionGetter() {
        return impl().setZeroSolutionGetter();
    }

    
    void setStatusSolutionGetter(IloBool status) {
        return impl().setStatusSolutionGetter(status);
    }

    
    void setStatusSolutionGetter(IloBool status, IloBool warnObsolete) {
        return impl().setStatusSolutionGetter(status,warnObsolete);
    }

    
    void setMIPInfoSolutionGetter(IloCplex::Callback cb) {
        IloCplex::MIPCallbackI* cbi = cb.getImpl()->getType()==IloCplex::Callback::MIPInfo ? static_cast<IloCplex::MIPCallbackI*>(cb.getImpl()) : 0;
        ASSERT_IMPL_OBJ(cbi);
        return impl().setMIPInfoSolutionGetter(*cbi);
    }

    
    void setZeroDataSource() {
        return impl().setZeroDataSource();
    }

    
    inline void setSolutionGetter(IloOplSolutionGetter getter);

    
    inline IloOplSolutionGetter getSolutionGetter() const;

    
    inline void resetSolutionGetter();

    
    inline void addResultPublisher(IloOplResultPublisher publisher);

    
	inline IloBool hasPublishers() const;

    
	inline void registerCustomDataHandler(const char* customId, IloOplCustomDataHandler handler);

    
	inline void unregisterCustomDataHandler(const char* customId);

    
    inline IloOplScriptExpression makeScriptExpression(const char* name, IloStringArray paramNames, const char* code);

    
    IloConstraint getConstraintElementItem(const char* name, IloOplDataElements indices) {
        return impl().getConstraintElementItem(name,static_cast<const IloOplDataElementsI&>(indices.impl()));
    }

    
    IloModel getModel() {
        return &impl().getModel();
    }

    
    IloModel getOuterModel() {
        return &impl().getOuterModel();
    }

    
    IloObjective getObjective() {
        return impl().getObjective();
    }

    
    void convertAllIntVars() {
        impl().convertAllIntVars();
    }

    
    void unconvertAllIntVars() {
        impl().unconvertAllIntVars();
    }

    
    IloCplex getCplex() const {
        return &impl().getCplex();
    }

    
    IloCP getCP() const;

  
  IloBool hasCplex() const {
      return impl().hasCplex();
  }

  
  IloBool hasCP() const {
      return impl().hasCP();
  }

    
    IloBool isUsingCplex() const {
        return hasCplex();
    }

    
    IloBool isUsingCP() const {
        return hasCP();
    }
    
    void generate() {
        impl().generate();
    }

    
    inline IloBool isGenerated() const;

    
    inline void generate(IloOplLabelCallback callback);

    
	void loadDataOnly() {
		impl().loadDataOnly();
	}

    
    inline void processDecisionExpr(IloNumExprArg expr, IloOplDecisionExprCallback callback);

    
    inline void processDecisionExprSolution(const char* name, IloOplDecisionExprSolutionCallback callback);

    
    IloNum evaluateConstraintLeft(IloMapIndexArray indexArray, IloMapIndexArray valueArray, IloConstraint ct) {
        return impl().evaluateConstraintLeft(indexArray,valueArray,ct.getImpl());
    }

    
    IloNum evaluateConstraintMid(IloMapIndexArray indexArray, IloMapIndexArray valueArray, IloConstraint ct) {
        return impl().evaluateConstraintMid(indexArray,valueArray,ct.getImpl());
    }

    
    IloNum evaluateConstraintRight(IloMapIndexArray indexArray, IloMapIndexArray valueArray, IloConstraint ct) {
        return impl().evaluateConstraintRight(indexArray,valueArray,ct.getImpl());
    }

    
    void postProcess() {
        impl().postProcess();
    }

   
    void warnNeverUsedElements() {
        impl().warnNeverUsedElements();
    }



    
    IloInt main() {
        return impl().main();
    }

    
    void printExternalData(std::ostream& os) {
        impl().printExternalData(os);
    }

    
    void printExternalData(std::ostream& os, const char* title) {
        impl().printExternalData(os,title);
    }



    
    void printInternalData(std::ostream& os) {
        impl().printInternalData(os);
    }

    
    void printInternalData(std::ostream& os, const char* title) {
        impl().printInternalData(os,title);
    }

    
    void printSolution(std::ostream& os) {
        impl().printSolution(os);
    }

    
    IloOplDataElements makeDataElements() const {
        return impl().makeDataElements();
    }

    
    inline IloOplConflictIterator getConflictIterator() const;

    
    inline IloOplRelaxationIterator getRelaxationIterator() const;

    
    inline IloBool setPoolSolution(IloInt solId);

    
    inline IloBool tuneParam(IloCplex::ParameterSet fixedSet);

    
    inline IloBool tuneParam(IloCplex::ParameterSet fixedSet, IloCplex::ParameterSet resultSet);

	
	void registerDisposeListener(IloOplDisposeListener d);
    
	void unregisterDisposeListener(IloOplDisposeListener d);

    
     void setProgressListener(IloOplProgressListener l);

    
     void removeProgressListener(IloOplProgressListener l);
    
     IloOplProgressListener getProgressListener();

   
   void saveResultsInText(const char* filePath);

private:
	void checkForMaaS(); 
};



class ILOOPL_EXPORTED IloOplResultPublisherBaseI: public IloOplResultPublisherI {
protected:

    
    IloOplResultPublisherBaseI(IloEnv env):IloOplResultPublisherI(env.getImpl()) {
    }
public:

    
    virtual ~IloOplResultPublisherBaseI() {
    }

    
    const IloOplModel getOplModel() const {
        return &IloOplResultPublisherI::getOplModel();
    }

    
    const IloOplElement getElement(const char* name) const {
        return &IloOplResultPublisherI::getElement(name);
    }

    
    IloOplErrorHandler getErrorHandler() const {
        return &IloOplResultPublisherI::getErrorHandler();
    }

    
    virtual void publish() =0;

    
    virtual const char* getResultPublisherName() const =0;
};



class ILOOPL_EXPORTED IloOplResultPublisher {
	HANDLE_DECL_OPL(IloOplResultPublisher)
public:
    
    IloOplResultPublisher(IloOplResultPublisherBaseI* impl):_impl(impl) {
    }

    
    const char* getResultPublisherName() const {
        return impl().getResultPublisherName();
    }
};



class ILOOPL_EXPORTED IloOplSolutionGetterBaseI: public IloOplSolutionGetterI {
protected:
    
    IloOplSolutionGetterBaseI(IloEnv env, IloOplModel opl):IloOplSolutionGetterI(env.getImpl(),opl.impl()) {
    }
public:
    
    virtual ~IloOplSolutionGetterBaseI() {
    }

    
    using IloOplSolutionGetterI::getWarnObsolete;

    
    virtual IloBool isSolutionAvailable() const =0;

    
    virtual IloInt getIntValue(IloIntVar var) const =0;

    
    virtual IloNum getNumValue(IloNumVar var) const =0;
};



class ILOOPL_EXPORTED IloOplSolutionGetter {
	HANDLE_DECL_OPL(IloOplSolutionGetter)
public:
    
    IloOplSolutionGetter(IloOplSolutionGetterBaseI* impl):_impl(impl) {
    }
    
    IloBool isSolutionAvailable() const {
        return impl().isSolutionAvailable();
    }
    
    IloBool isIntermediateSolution() const {
        return impl().isIntermediateSolution();
    }
    
    void printSolutionTitle(std::ostream& os) const {
        impl().printSolutionTitle(os);
    }

    
    IloBool hasObjValue() const {
        return impl().hasObjValue();
    }
    
    IloNum getObjValue() const {
        return impl().getObjValue();
    }

    
    IloInt evalIntExpr(IloIntExprArg expr) const {
        return impl().evalIntExpr(expr);
    }
    
    IloNum evalNumExpr(IloNumExprArg expr) const {
        return impl().evalNumExpr(expr);
    }
};



class ILOOPL_EXPORTED IloOplScriptThunk {
    HANDLE_DECL_OPL(IloOplScriptThunk)
public:
    
	void updateSharedContext(IloStringArray names, IloMapIndexArray values, IloBool reset) const {
        impl().updateSharedContext(names,values,reset);
    }

    
    void execute() const {
        return impl().execute();
    }

    
    const char* eval() const {
        return impl().eval();
    }

    
    IloInt evalInt() const {
        return impl().evalInt();
    }

    
	IloNum evalNum() const {
        return impl().evalNum();
    }

    
	IloOplObject evalObject() const {
        return impl().evalObject();
    }

    
	const char* getCode() const {
        return impl().getCode();
    }
};



class ILOOPL_EXPORTED IloOplScriptExpression: public IloOplScriptThunk {
    HANDLE_DECL_SUB_OPL(IloOplScriptExpression,IloOplScriptThunk)
public:
    
    const char* eval(IloOplDataElements paramsL) const{
        return impl().eval(static_cast<const IloOplDataElementsI&>(paramsL.impl()));
    }

    
    IloInt evalInt(IloOplDataElements paramsL) const{
        return impl().evalInt(static_cast<const IloOplDataElementsI&>(paramsL.impl()));
    }

    
    IloNum evalNum(IloOplDataElements paramsL) const{
        return impl().evalNum(static_cast<const IloOplDataElementsI&>(paramsL.impl()));
    }

    
    const char* eval(IloStringArray paramNames, IloMapIndexArray values) const {
        return impl().eval(paramNames,values);
    }

    
    IloInt evalInt(IloStringArray paramNames, IloMapIndexArray values) const {
        return impl().evalInt(paramNames,values);
    }

    
    IloNum evalNum(IloStringArray paramNames, IloMapIndexArray values) const {
        return impl().evalNum(paramNames,values);
    }

    
    IloOplObject evalObject(IloStringArray paramNames, IloMapIndexArray values) const {
        return impl().evalObject(paramNames,values);
    }
};



class ILOOPL_EXPORTED IloOplRelaxationIterator {
    HANDLE_DECL_OPL(IloOplRelaxationIterator)
public:
    
    explicit IloOplRelaxationIterator(IloOplModel model) {
        // TODO support more engines with relaxation capabilities
        _impl = new (model.getEnv()) IloOplCplexFeasOptIteratorI(model.impl());
    }

    
    IloInt printRelaxation(std::ostream& os) {
        return impl().printRelaxation(os);
    }

    
    virtual IloNum getLB() const {
      return impl().currentLB();
    }
    
    virtual IloNum getUB() const {
      return impl().currentUB();
    }
    
    virtual IloNum getRelaxedLB() const {
      return impl().currentRelaxedLB();
    }
    
    virtual IloNum getRelaxedUB() const {
      return impl().currentRelaxedUB();
    }
    
    virtual IloConstraint getConstraint() const {
       return impl().currentRelaxedConstraint();
    }

    
    void attach(IloConstraintMap cts, IloNumMap prefs) {
      impl().attach(cts, prefs);
    }

    
    void attach(IloConstraint ct, IloNum pref) {
       impl().attach(ct, pref);
    }
    
    
    IloBool hasNext() {
       return impl().hasRelaxed();
    }
    
    void next() {
      impl().nextRelaxed();
    }
    
    void first() {
      impl().firstRelaxed();
    }
};



class ILOOPL_EXPORTED IloOplConflictIterator {
    HANDLE_DECL_OPL(IloOplConflictIterator)
public:
    
    explicit IloOplConflictIterator(IloOplModel model) {
        
       // Support CP Optimizer engine
       // OPL team: please check this code
       if (model.hasCplex()) {
         _impl = new (model.getEnv()) IloOplCplexConflictIteratorI(model.impl());
       } else {
         _impl = new (model.getEnv()) IloOplCPConflictIteratorI(model.impl());
       }
       
    }

    
    IloInt printConflict(std::ostream& os) {
        return impl().printConflict(os);
    }

    
    void excludeConflict() {
        return impl().excludeConflict();
    }

    
    void attach(IloConstraintMap cts, IloNumMap prefs) {
       impl().attach(cts, prefs);
    }

    
    void attach(IloConstraint ct, IloNum pref) {
      impl().attach(ct, pref);
    }

    
    void clearAttachements() {
      impl().clearAttachments();
    }
    
    
    IloBool hasNext() const {
       return impl().isInConflict();
    }
    
    void next() {
      impl().nextInConflict();
    }
    
    void first() {
      impl().firstInConflict();
    }
    
    
        
    IloExtractable getExtractable() const {
      return impl().currentInConflict();
    }

    
    IloInt getStatus() {
        return impl().currentConflictStatus();
    }
};



class ILOOPL_EXPORTED IloOplCplexBasis {
    HANDLE_DECL_OPL(IloOplCplexBasis)

public:
    
    explicit IloOplCplexBasis(IloEnv env) {
        _impl = new (env) IloOplCplexBasisI(env.getImpl());
    }

    
    IloBool getBasis(IloCplex cplex) {
        return impl().getBasis(cplex.getImpl());
    }

    
    IloBool setBasis(IloCplex cplex) {
        return impl().setBasis(cplex.getImpl());
    }

    
    IloInt getStatus() const {
        return impl().getStatus();
    }

    
    IloInt getNrows() const {
        return impl().getNrows();
    }

    
    IloInt getNcols() const {
        return impl().getNcols();
    }

    
    void attach(IloNumVarMap vars, IloIntMap statuses) {
      impl().attach(vars, statuses);
    }
};



class ILOOPL_EXPORTED IloOplCplexVectors {
    HANDLE_DECL_OPL(IloOplCplexVectors)

public:
    
    explicit IloOplCplexVectors(IloEnv env) {
        _impl = new (env) IloOplCplexVectorsI(env.getImpl());
    }


    
    void deleteStart(IloCplex cplex) {
        impl().deleteStart(cplex.getImpl());
    }
 
    
    void clear(){
        impl().clear();
    }

    
    IloBool getStart(IloCplex cplex) {
        return impl().getStart(cplex.getImpl());
    }

    
    IloBool setStart(IloCplex cplex) {
        return impl().setStart(cplex.getImpl());
    }

    
    IloInt getStatus() const {
        return impl().getStatus();
    }

    
    IloInt getNrows() const {
        return impl().getNrows();
    }

    
    IloInt getNcols() const {
        return impl().getNcols();
    }

    
    void attach(IloNumVarMap vars, IloNumMap values) {
      impl().attach(vars, values);
    }

    
    void attach(IloIntVarMap vars, IloIntMap values) {
      impl().attach(vars, values);
    }
};



class ILOOPL_EXPORTED IloOplLabelCallbackBaseI: public IloOplLabelCallbackI {
protected:
    
    IloOplLabelCallbackBaseI(IloEnv env, IloBool enableScripting):IloOplLabelCallbackI(env.getImpl(),enableScripting) {
    }
public:
    
    virtual ~IloOplLabelCallbackBaseI() {
    }

    
	virtual void main(const char* label) =0;

    
	IloBool hasChangedIndexArray() const {
		return IloOplLabelCallbackI::hasChangedIndexArray();
	}
    
	IloMapIndexArray currentIndexArray() const {
		return IloOplLabelCallbackI::currentIndexArray();
	}

    
	IloBool hasChangedIndexNameArray() const {
		return IloOplLabelCallbackI::hasChangedIndexNameArray();
	}

    
	IloStringArray currentIndexNameArray() const {
		return IloOplLabelCallbackI::currentIndexNameArray();
	}

    
	IloBool hasChangedIndexValueArray() const {
		return IloOplLabelCallbackI::hasChangedIndexValueArray();
	}

    
	IloMapIndexArray currentIndexValueArray() const {
		return IloOplLabelCallbackI::currentIndexValueArray();
	}

    
	const char* evaluate(IloOplScriptExpression expr) const{
		return IloOplLabelCallbackI::evaluate(expr.impl());
	}

    
	IloNum evaluateNum(IloOplScriptExpression expr) const{
		return IloOplLabelCallbackI::evaluateNum(expr.impl());
	}

    
	IloInt evaluateInt(IloOplScriptExpression expr) const {
		return IloOplLabelCallbackI::evaluateInt(expr.impl());
	}

    
	IloOplObject evaluateObject(IloOplScriptExpression expr) const {
		return IloOplLabelCallbackI::evaluateObject(expr.impl());
	}

    
    IloConstraint currentConstraint() const {
        return IloOplLabelCallbackI::currentConstraint();
    }

    
	IloBool isInForAll() const {
        return IloOplLabelCallbackI::isInForAll();
    }

    
	IloBool isLogical() const {
        return IloOplLabelCallbackI::isLogical();
    }

    
	IloForAllRange currentForAllRange() const {
        return IloOplLabelCallbackI::currentForAllRange();
    }

    
	IloOplObject evaluate(IloOplObject arg) const {
        return IloOplLabelCallbackI::evaluate(arg);
    }
};



class ILOOPL_EXPORTED IloOplLabelCallback {
	HANDLE_DECL_OPL(IloOplLabelCallback)
public:
    
    IloOplLabelCallback(IloOplLabelCallbackBaseI* impl):_impl(impl) {
    }
};



class ILOOPL_EXPORTED IloOplCustomDataHandlerBaseI: public IloOplCustomDataHandlerI {
protected:
    
    IloOplCustomDataHandlerBaseI(IloEnv env):IloOplCustomDataHandlerI(env.getImpl()) {
    }
public:

    
	virtual ~IloOplCustomDataHandlerBaseI() {
	}

    
	virtual void handleConnection(const char* connId, const char* subId, const char* spec) =0;
    
	virtual void handleReadElement(const char* connId, const char* name, const char* spec) =0;
    
	virtual void handlePublishElement(const char* connId, const char* name, const char* spec) =0;
    
	virtual IloBool handleInvoke(const char* name, const char* funcname) =0;
    
	virtual void closeConnections() {
	}

    
	IloOplDataHandler getDataHandler() {
		return &IloOplCustomDataHandlerI::getDataHandler();
	}

    
	void raiseError(const char* message) {
		IloOplCustomDataHandlerI::raiseError(message);
	}
};


class ILOOPL_EXPORTED IloOplCustomDataHandler {
	HANDLE_DECL_OPL(IloOplCustomDataHandler)
public:

    
    IloOplCustomDataHandler(IloOplCustomDataHandlerBaseI* impl):_impl(impl) {
    }
};



class ILOOPL_EXPORTED IloOplDecisionExprCallbackBaseI: public IloOplDecisionExprCallbackI {
protected:
    
    IloOplDecisionExprCallbackBaseI(IloEnv env, IloBool enableScripting):IloOplDecisionExprCallbackI(env.getImpl(),enableScripting) {
    }

public:
    
    virtual ~IloOplDecisionExprCallbackBaseI() {
    }

    
    virtual IloBool startDecisionExpr(const char* dexprName, IloNum coef, IloNumExprArg expr) =0;

    
    virtual IloBool startSum(IloNum coef, IloNumExprArg expr) =0;

    
    virtual void addTerm(IloNum coef, IloNumExprArg expr) =0;

    
    virtual void endSum() =0;

    
    virtual void endDecisionExpr(const char* dexprName) =0;
};



class ILOOPL_EXPORTED IloOplDecisionExprSolutionCallbackBaseI: public IloOplDecisionExprSolutionCallbackI {
protected:
    
    IloOplDecisionExprSolutionCallbackBaseI(IloEnv env, IloBool enableScripting):IloOplDecisionExprSolutionCallbackI(env.getImpl(),enableScripting) {
    }

    virtual IloBool startDecisionExpr(const char* dexprName, IloNum coef, IloNumExprArg expr) {
        return IloOplDecisionExprSolutionCallbackI::startDecisionExpr(dexprName,coef,expr);
    }
    virtual IloBool startSum(IloNum coef, IloNumExprArg expr) {
        return IloOplDecisionExprSolutionCallbackI::startSum(coef,expr);
    }
    virtual void addTerm(IloNum coef, IloNumExprArg dexpr) {
        return IloOplDecisionExprSolutionCallbackI::addTerm(coef,dexpr);
    }

public:
    
    virtual ~IloOplDecisionExprSolutionCallbackBaseI() {
    }

    
    virtual IloBool startDecisionExpr(const char* dexprName, IloNum value) =0;

    
    virtual IloBool startSum(IloNum coef) =0;

    
    virtual void addTerm(IloNum value) =0;
    
    virtual void endSum() =0;

    
    virtual void endDecisionExpr(const char* dexprName) =0;

    
    IloMapIndexArray currentIndexArray() const {
        return IloOplDecisionExprCallbackI::currentIndexArray();
    }
    
	IloOplObject evaluate(IloOplObject index) const {
        return IloOplDecisionExprCallbackI::evaluate(index);
    }

    
    IloBool hasChangedIndexNameArray() const {
        return IloOplDecisionExprCallbackI::hasChangedIndexNameArray();
    }
    
	IloStringArray currentIndexNameArray() const {
        return IloOplDecisionExprCallbackI::currentIndexNameArray();
    }

    
	IloBool hasChangedIndexValueArray() const {
        return IloOplDecisionExprCallbackI::hasChangedIndexValueArray();
    }

    
	IloMapIndexArray currentIndexValueArray() const {
        return IloOplDecisionExprCallbackI::currentIndexValueArray();
    }

    
	const char* evaluate(IloOplScriptExpression expr) const {
        return IloOplDecisionExprCallbackI::evaluate(expr.impl());
    }

    
	IloNum evaluateNum(IloOplScriptExpression expr) const {
        return IloOplDecisionExprCallbackI::evaluateNum(expr.impl());
    }

    
	IloInt evaluateInt(IloOplScriptExpression expr) const {
        return IloOplDecisionExprCallbackI::evaluateInt(expr.impl());
    }

    
    IloOplObject evaluateObject(IloOplScriptExpression expr) const {
        return IloOplDecisionExprCallbackI::evaluateObject(expr.impl());
    }
};


class ILOOPL_EXPORTED IloOplDecisionExprCallback {
	HANDLE_DECL_OPL(IloOplDecisionExprCallback)
public:
	
    IloOplDecisionExprCallback(IloOplDecisionExprCallbackBaseI* impl):_impl(impl) {
    }
};



class ILOOPL_EXPORTED IloOplDecisionExprSolutionCallback {
	HANDLE_DECL_OPL(IloOplDecisionExprSolutionCallback)
public:
    
    IloOplDecisionExprSolutionCallback(IloOplDecisionExprSolutionCallbackBaseI* impl):_impl(impl) {
    }
};


class ILOOPL_EXPORTED IloOplTuningCallbackBaseI: public IloOplTuningCallbackI {
protected:
    
    IloOplTuningCallbackBaseI(IloEnv env):IloOplTuningCallbackI(env.getImpl()) {
    }

public:
    
    virtual ~IloOplTuningCallbackBaseI() {
    }

    
    virtual void notifyGenerate(IloOplModel opl, const char*) =0;
    
    virtual void notifyStart(IloCplex cplex) =0;
    
    virtual void notifyProgress() =0;
    
    virtual void notifyEnd(IloCplex cplex) =0;

    
    IloNum getProgress() const {
        return IloOplTuningCallbackI::getProgress();
    }
    
    IloNum getRemaining() const {
        return IloOplTuningCallbackI::getRemaining();
    }
};


class ILOOPL_EXPORTED IloOplTuningCallback {
	HANDLE_DECL_OPL(IloOplTuningCallback)
public:
    
    IloOplTuningCallback(IloOplTuningCallbackBaseI* impl):_impl(impl) {
    }
};

// ---------------------------------------------------------------------------

inline
void IloOplModel::setSolutionGetter(IloOplSolutionGetter getter) {
    impl().setSolutionGetter(getter.impl());
}

inline
IloOplSolutionGetter IloOplModel::getSolutionGetter() const {
    return &impl().getSolutionGetter();
}

inline
void IloOplModel::resetSolutionGetter() {
    impl().resetSolutionGetter();
}

inline
IloBool IloOplModel::hasPublishers() const {
    return impl().hasPublishers();
}

inline
void IloOplModel::addResultPublisher(IloOplResultPublisher publisher) {
    impl().addResultPublisher(&publisher.impl());
}

inline
void IloOplModel::registerCustomDataHandler(const char* customId, IloOplCustomDataHandler handler) {
	impl().registerCustomDataHandler(customId,handler.impl());
}

inline
void IloOplModel::unregisterCustomDataHandler(const char* customId) {
	impl().unregisterCustomDataHandler(customId);
}

inline
IloOplScriptExpression IloOplModel::makeScriptExpression(const char* name, IloStringArray paramNames, const char* code) {
    return impl().makeScriptExpression(name,paramNames,code);
}

inline
IloBool IloOplModel::isGenerated() const {
    return impl().isGenerated();
}

inline
void IloOplModel::generate(IloOplLabelCallback callback) {
    impl().generate(callback.impl());
}

inline
void IloOplModel::processDecisionExpr(IloNumExprArg expr, IloOplDecisionExprCallback callback) {
    impl().processDecisionExpr(expr.getImpl(),callback.impl());
}

inline
void IloOplModel::processDecisionExprSolution(const char* name, IloOplDecisionExprSolutionCallback callback) {
    impl().processDecisionExprSolution(name,callback.impl());
}

inline
IloOplRelaxationIterator IloOplModel::getRelaxationIterator() const {
    return &impl().getRelaxationIterator();
}
inline
IloOplConflictIterator IloOplModel::getConflictIterator() const {
    return &impl().getConflictIterator();
}
inline
IloBool IloOplModel::setPoolSolution(IloInt solId) {
  return impl().setPoolSolution(solId);
}
inline
IloBool IloOplModel::tuneParam(IloCplex::ParameterSet fixedSet, IloCplex::ParameterSet resultSet) {
  ASSERT_IMPL_OBJ(fixedSet.getImpl());
  ASSERT_IMPL_OBJ(resultSet.getImpl());
  return impl().tuneParam(*fixedSet.getImpl(), resultSet.getImpl());
}
inline
IloBool IloOplModel::tuneParam(IloCplex::ParameterSet fixedSet) {
  ASSERT_IMPL_OBJ(fixedSet.getImpl());
  return impl().tuneParam(*fixedSet.getImpl(),0);
}


class IloOplDisposeListenerI : public IloEnvObjectI{
public:
	IloOplDisposeListenerI(IloEnv env) : IloEnvObjectI(env.getImpl()){}
	virtual ~IloOplDisposeListenerI(){}
	
	virtual void dispose(IloOplModel opl) = 0;
};



class IloOplPostProcessListenerI : public IloEnvObjectI{
public:
	IloOplPostProcessListenerI(IloEnv env) : IloEnvObjectI(env.getImpl()){}
	virtual ~IloOplPostProcessListenerI(){}
	
	virtual void startPostProcess() = 0;
	virtual void endPostProcess() = 0;
};

#ifdef _WIN32
#pragma pack(pop)
#endif

#endif
