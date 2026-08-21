// LauncherWindow.h
//
// Main window for mars_launcher, a standalone Qt6 Widgets desktop launcher
// for the MARS / ReloPush executables. It reads ui/options_spec.json at
// startup, builds a form from it, and constructs + runs a command line for
// the user-selected executable (Advanced mode). It also reads
// ui/basic_refinements.json to drive a simplified instance/refinement picker
// (Basic mode) for the two most common workflows: refining a greedy
// allocation via phastar_push_demo (LNS / online DQN), or scoring a
// pretrained policy via script/pg/propose_best_plan.py.
//
// See ui/README.md for build/run instructions, ui/options_spec.json for the
// Advanced-mode schema, and ui/basic_refinements.json for the Basic-mode
// schema.

#pragma once

#include <QMainWindow>
#include <QVector>
#include <QString>
#include <QStringList>
#include <QJsonValue>
#include <QProcess>

QT_BEGIN_NAMESPACE
class QComboBox;
class QTabWidget;
class QPlainTextEdit;
class QCheckBox;
class QSpinBox;
class QDoubleSpinBox;
class QLineEdit;
class QPushButton;
class QFormLayout;
class QLabel;
QT_END_NAMESPACE

// One entry from the "options" array in options_spec.json.
struct Option {
    QString flag;            // e.g. "--num-robots=", or "" for positional args
    QString name;
    QString type;            // "int" | "float" | "string" | "file" | "enum" | "bool"
    bool takesValue = true;
    QString valueSeparator;  // "=" or " " (empty for bare bool flags)
    QJsonValue defaultValue; // may be null
    QStringList choices;     // empty if none specified
    QString description;
    QString group;
    QStringList appliesTo;
    QStringList aliases;
    bool hasMin = false;
    double min = 0.0;
    bool hasMax = false;
    double max = 0.0;
    QString notes;
    int positionalIndex = -1; // parsed from notes "positional_index=N"; -1 if not positional

    bool isPositional() const { return flag.isEmpty(); }
};

// One entry from the "executables" array in options_spec.json.
struct ExeInfo {
    QString name;
    QString path;
    QString description;
};

// Live widgets for a single option row in the form. Populated by
// rebuildForm() and consulted by buildAdvancedCommandArgv()/rebuildPreview().
struct OptionRowWidget {
    const Option *opt = nullptr;
    QCheckBox *enableCheck = nullptr; // null for positional args (always emitted)
    QComboBox *combo = nullptr;       // enum / file (editable) editors
    QSpinBox *spin = nullptr;         // int editor
    QDoubleSpinBox *doubleSpin = nullptr; // float editor
    QLineEdit *line = nullptr;        // string editor
    QPushButton *browse = nullptr;    // file editor's Browse... button
};

// One entry from basic_refinements.json's "families" array.
struct BasicFamily {
    QString id;       // e.g. "boss8"
    QString b64Name;  // e.g. "ReloPush-BOSS_8_objects.txt"
    int indexMin = 0;
    int indexMax = 99;
};

// One entry from basic_refinements.json's "refinements" array.
struct BasicRefinement {
    QString id;
    QString name;
    QString runner;          // "cpp" | "propose_verify"
    QStringList extraFlags;  // cpp runner only, e.g. ["--search-mode=lns"]
    QString model;           // propose_verify runner only, path relative to repo root
    int defaultK = 40;
    bool supportsNumRobots = false;
    bool supportsVisualize = false;
};

class LauncherWindow : public QMainWindow {
    Q_OBJECT
public:
    // specPath: resolved path to options_spec.json (see main.cpp for the
    // resolution order). basicSpecPath: resolved path to
    // basic_refinements.json; if empty, a file named "basic_refinements.json"
    // next to specPath is tried. If loading either fails, the corresponding
    // *SpecLoadOk() returns false and *LastError() explains why.
    explicit LauncherWindow(const QString &specPath,
                             const QString &basicSpecPath = QString(),
                             QWidget *parent = nullptr);

    // ---- Advanced-mode (options_spec.json) ----
    bool specLoadOk() const { return specLoaded_; }
    const QString &lastError() const { return lastError_; }

    // True iff every option's "type" field is a type we know how to render.
    // Used by --selftest.
    bool allOptionTypesKnown() const;

    QStringList executableNames() const;

    // Selects an executable by name (as in the Advanced page's "Executable:"
    // combo) and rebuilds the form for it. Returns false if not found.
    bool selectExecutable(const QString &exeName);

    // Builds the argv that would be passed to QProcess::start("env", argv)
    // for the CURRENTLY ACTIVE mode (Basic or Advanced) and its current
    // selections/display mode.
    QStringList buildCommandArgv() const;

    // Renders buildCommandArgv() (plus the literal "env" program name) as a
    // single shell-escaped, copy-pasteable line prefixed with "cd <cwd> && ".
    QString buildPreviewLine() const;

    // Path to options_spec.json's declared repo_root, used for on-disk
    // existence checks and as the QFileDialog starting directory.
    const QString &repoRoot() const { return repoRoot_; }

    // ---- Basic-mode (basic_refinements.json) + selftest hooks ----
    bool basicSpecLoadOk() const { return basicSpecLoaded_; }
    const QString &basicLastError() const { return basicLastError_; }

    // True iff every refinement's "runner" field is "cpp" or "propose_verify".
    bool allBasicRunnersKnown() const;

    QStringList refinementIds() const;
    QString runnerForRefinement(const QString &refId) const;
    QString modelForRefinement(const QString &refId) const; // empty for cpp runners

    void switchToBasicMode();
    void switchToAdvancedMode();

    // Selects a family by id (e.g. "boss8") in the Basic page and refreshes
    // the index spin box's range. Returns false if not found.
    bool selectBasicFamily(const QString &familyId);
    void setBasicIndex(int index);
    void setBasicNumRobots(int n);

    // Selects a refinement by id in the Basic page and refreshes the
    // dependent controls (num-robots enable state, K visibility, visualize
    // enable state). Returns false if not found.
    bool selectBasicRefinement(const QString &refId);
    void setBasicK(int k);
    void setBasicVisualize(bool on);

    // Basic-mode helper: argv for the automatic `--play-result=` follow-up
    // step that fires after a propose_verify run finishes successfully, when
    // Visualize is checked (empty QStringList if the current selection
    // doesn't qualify -- cpp refinements, propose_verify with Visualize
    // unchecked, or a refinement that doesn't support visualize at all).
    // Exposed publicly so --selftest can assert on it without actually
    // running the two-step chain.
    QStringList buildBasicFollowupArgv() const;

private slots:
    void onExecutableChanged(int index);
    void onDisplayChanged(int index);
    void rebuildPreview();
    void onRun();
    void onStop();
    void onCopyCommand();
    void onClearOutput();
    void onProcessReadyRead();
    void onProcessFinished(int exitCode, QProcess::ExitStatus status);
    void onProcessErrorOccurred(QProcess::ProcessError error);
    void onModeChanged(int index);
    void onBasicFamilyChanged(int index);
    void onBasicRefinementChanged(int index);

private:
    // ---- setup ----
    bool loadSpec(const QString &specPath);
    bool loadBasicSpec(const QString &basicSpecPath);
    void setupUi();
    QWidget *buildAdvancedPage();
    QWidget *buildBasicPage();
    void populateExecutableCombo();
    void rebuildForm();
    void addOptionRow(QFormLayout *form, const Option &opt);
    void connectRowLiveSignals(const OptionRowWidget &row);

    // ---- helpers (shared) ----
    const ExeInfo *currentExecutable() const;
    bool isHeadlessDisplay() const;
    QString shellQuote(const QString &s) const;
    // "env" "-i" HOME=... USER=... PATH=... plus the Display-combo-driven
    // QT_QPA_PLATFORM=(offscreen|xcb)[+DISPLAY/XAUTHORITY] block -- the exact
    // env-wrapper prefix shared by buildAdvancedCommandArgv(), the cpp branch
    // of buildBasicCommandArgv(), and buildPlayResultArgv() (the visualize
    // follow-up step), so all three stay in lockstep with the Display combo.
    QStringList envWrapperPrefix() const;
    // Value currently held by an option's editor, formatted as CLI text.
    // Returns false if the option is disabled or (for value-taking options)
    // its editor is empty.
    bool currentOptionValue(const OptionRowWidget &row, QString *valueOut) const;
    QString firstExistingChoice(const QStringList &choices) const;
    QStringList buildAdvancedCommandArgv() const;

    // ---- helpers (basic mode) ----
    bool isBasicMode() const;
    const BasicFamily *currentBasicFamily() const;
    const BasicRefinement *currentBasicRefinement() const;
    QString basicSequenceRelPath(const BasicFamily &fam, int index) const;
    QStringList buildBasicCommandArgv() const;
    // Non-blocking warning text (empty if nothing is missing) about
    // sequence/table/model files the currently-selected Basic run needs.
    QString basicExistenceWarning() const;
    void updateBasicControlsForRefinement();
    // Directory for Basic-mode scratch files (--out-best, --result-out /
    // --play-result=): the job tmp dir if present, else QDir::tempPath().
    QString basicTmpDir() const;
    // RESULT_PATH for the currently-selected family/index: <basicTmpDir()>/
    // mars_launcher_result_<family>_ind<index>.scn.b64. Empty if no family
    // is selected.
    QString basicResultPath() const;
    // env-wrapped `phastar_push_demo --play-result=<resultPath>` argv (same
    // env-wrapper + Display-combo logic as a cpp refinement's run). Empty if
    // resultPath is empty or phastar_push_demo isn't in the executables list.
    QStringList buildPlayResultArgv(const QString &resultPath) const;

    // ---- data model: advanced (options_spec.json) ----
    QString specPath_;
    QString repoRoot_;
    QString envCwd_;
    QString envWrapperNote_;
    bool specLoaded_ = false;
    QString lastError_;
    QVector<Option> allOptions_;
    QVector<ExeInfo> executables_;

    // ---- data model: basic (basic_refinements.json) ----
    QString basicSpecPath_;
    bool basicSpecLoaded_ = false;
    QString basicLastError_;
    QVector<BasicFamily> basicFamilies_;
    QVector<BasicRefinement> basicRefinements_;
    QString basicSequenceDir_;
    QString basicSequencePattern_;
    QString basicTablesDir_;
    QString basicProposeScript_;
    int basicNumRobotsDefault_ = 3;
    int basicNumRobotsMin_ = 1;
    int basicNumRobotsMax_ = 3;
    QString basicNumRobotsNote_;

    // ---- UI: shared (mode switch, display, preview, run controls, output) ----
    QTabWidget *modeTabs_ = nullptr; // tab 0 = Basic, tab 1 = Advanced
    QComboBox *displayCombo_ = nullptr;
    QPlainTextEdit *previewEdit_ = nullptr;
    QPlainTextEdit *outputEdit_ = nullptr;
    QPushButton *runButton_ = nullptr;
    QPushButton *stopButton_ = nullptr;
    QPushButton *copyButton_ = nullptr;
    QPushButton *clearButton_ = nullptr;

    // ---- UI: Advanced page ----
    QComboBox *exeCombo_ = nullptr;
    QTabWidget *tabWidget_ = nullptr; // per-group option tabs
    QVector<OptionRowWidget> currentRows_; // rebuilt every time the form is rebuilt

    // ---- UI: Basic page ----
    QComboBox *basicFamilyCombo_ = nullptr;
    QSpinBox *basicIndexSpin_ = nullptr;
    QComboBox *basicRefinementCombo_ = nullptr;
    QSpinBox *basicNumRobotsSpin_ = nullptr;
    QLabel *basicKLabel_ = nullptr;
    QSpinBox *basicKSpin_ = nullptr;
    QCheckBox *basicVisualizeCheck_ = nullptr;
    QLabel *basicNoteLabel_ = nullptr;
    QLabel *basicWarningLabel_ = nullptr;

    QProcess *process_ = nullptr;

    // ---- Step A -> Step B chain (propose_verify + Visualize) ----
    // Non-empty iff a follow-up `--play-result=` process is queued to start
    // when the CURRENTLY RUNNING process (Step A, the propose_best_plan.py
    // call) finishes successfully. Set in onRun(), consumed (and cleared) in
    // onProcessFinished(); also cleared in onStop() so a user abort never
    // lets the follow-up fire.
    QStringList pendingFollowupArgv_;
    QString pendingFollowupPreview_;
    QString pendingFollowupResultPath_;
};
