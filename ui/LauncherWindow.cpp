// LauncherWindow.cpp
//
// See LauncherWindow.h for the class overview, ui/options_spec.json for the
// Advanced-mode option-catalog schema, and ui/basic_refinements.json for the
// Basic-mode refinement-catalog schema, both parsed here.

#include "LauncherWindow.h"

#include <QApplication>
#include <QClipboard>
#include <QComboBox>
#include <QCheckBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFont>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QRegularExpression>
#include <QScrollArea>
#include <QSpinBox>
#include <QSplitter>
#include <QTabWidget>
#include <QTextCursor>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>

namespace {
// Order in which group tabs are presented; Input always first as required.
const QStringList kGroupOrder = {
    "Input", "Robots", "Search", "DQN/RL", "Evaluation",
    "Export", "Output/Visualization", "Integration", "Replay"
};

bool jsonValuePresent(const QJsonValue &v) {
    return !v.isNull() && !v.isUndefined();
}
} // namespace

LauncherWindow::LauncherWindow(const QString &specPath, const QString &basicSpecPath,
                                 QWidget *parent)
    : QMainWindow(parent) {
    specLoaded_ = loadSpec(specPath);

    QString resolvedBasicPath = basicSpecPath;
    if (resolvedBasicPath.isEmpty())
        resolvedBasicPath = QFileInfo(specPath).absoluteDir().filePath("basic_refinements.json");
    basicSpecLoaded_ = loadBasicSpec(resolvedBasicPath);

    setupUi();

    if (specLoaded_) {
        populateExecutableCombo();
        rebuildForm();
    } else {
        if (outputEdit_) {
            outputEdit_->appendPlainText(
                QString("Failed to load option spec from '%1': %2")
                    .arg(specPath, lastError_));
        }
        QMessageBox::critical(this, "mars_launcher - spec load failed",
                               QString("Failed to load option spec:\n%1\n\n%2")
                                   .arg(specPath, lastError_));
    }

    if (!basicSpecLoaded_ && outputEdit_) {
        outputEdit_->appendPlainText(
            QString("Failed to load basic-mode refinement catalog from '%1': %2")
                .arg(resolvedBasicPath, basicLastError_));
    }

    rebuildPreview();
}

// ---------------------------------------------------------------------
// Spec loading: Advanced mode (options_spec.json)
// ---------------------------------------------------------------------

bool LauncherWindow::loadSpec(const QString &specPath) {
    specPath_ = specPath;
    QFile file(specPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        lastError_ = QString("could not open file: %1").arg(file.errorString());
        return false;
    }
    const QByteArray data = file.readAll();
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (doc.isNull() || parseError.error != QJsonParseError::NoError) {
        lastError_ = QString("JSON parse error: %1").arg(parseError.errorString());
        return false;
    }
    if (!doc.isObject()) {
        lastError_ = "spec root is not a JSON object";
        return false;
    }
    QJsonObject root = doc.object();

    repoRoot_ = root.value("repo_root").toString();

    for (const QJsonValue &v : root.value("executables").toArray()) {
        QJsonObject o = v.toObject();
        ExeInfo e;
        e.name = o.value("name").toString();
        e.path = o.value("path").toString();
        e.description = o.value("description").toString();
        if (!e.name.isEmpty()) executables_.append(e);
    }
    if (executables_.isEmpty()) {
        lastError_ = "spec has no executables";
        return false;
    }

    QJsonObject envObj = root.value("env").toObject();
    envCwd_ = envObj.value("cwd").toString();
    envWrapperNote_ = envObj.value("note").toString();
    if (envCwd_.isEmpty()) envCwd_ = repoRoot_;

    static const QRegularExpression positionalRe("positional_index=(\\d+)");

    for (const QJsonValue &v : root.value("options").toArray()) {
        QJsonObject o = v.toObject();
        Option opt;
        opt.flag = o.value("flag").toString();
        opt.name = o.value("name").toString();
        opt.type = o.value("type").toString();
        opt.takesValue = o.value("takes_value").toBool(true);
        opt.valueSeparator = o.value("value_separator").toString();
        opt.defaultValue = o.value("default");
        if (o.value("choices").isArray()) {
            for (const QJsonValue &c : o.value("choices").toArray())
                opt.choices << c.toVariant().toString();
        }
        opt.description = o.value("description").toString();
        opt.group = o.value("group").toString();
        if (o.value("applies_to").isArray()) {
            for (const QJsonValue &a : o.value("applies_to").toArray())
                opt.appliesTo << a.toString();
        }
        if (o.value("aliases").isArray()) {
            for (const QJsonValue &a : o.value("aliases").toArray())
                opt.aliases << a.toString();
        }
        if (jsonValuePresent(o.value("min"))) {
            opt.hasMin = true;
            opt.min = o.value("min").toDouble();
        }
        if (jsonValuePresent(o.value("max"))) {
            opt.hasMax = true;
            opt.max = o.value("max").toDouble();
        }
        opt.notes = o.value("notes").toString();
        if (!opt.notes.isEmpty()) {
            QRegularExpressionMatch m = positionalRe.match(opt.notes);
            if (m.hasMatch()) opt.positionalIndex = m.captured(1).toInt();
        }
        allOptions_.append(opt);
    }
    if (allOptions_.isEmpty()) {
        lastError_ = "spec has no options";
        return false;
    }
    return true;
}

bool LauncherWindow::allOptionTypesKnown() const {
    static const QStringList kKnownTypes = {"int", "float", "string", "file", "enum", "bool"};
    for (const Option &o : allOptions_) {
        if (!kKnownTypes.contains(o.type)) return false;
    }
    return true;
}

QStringList LauncherWindow::executableNames() const {
    QStringList names;
    for (const ExeInfo &e : executables_) names << e.name;
    return names;
}

// ---------------------------------------------------------------------
// Spec loading: Basic mode (basic_refinements.json)
// ---------------------------------------------------------------------

bool LauncherWindow::loadBasicSpec(const QString &basicSpecPath) {
    basicSpecPath_ = basicSpecPath;
    QFile file(basicSpecPath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        basicLastError_ = QString("could not open file: %1").arg(file.errorString());
        return false;
    }
    const QByteArray data = file.readAll();
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    if (doc.isNull() || parseError.error != QJsonParseError::NoError) {
        basicLastError_ = QString("JSON parse error: %1").arg(parseError.errorString());
        return false;
    }
    if (!doc.isObject()) {
        basicLastError_ = "basic spec root is not a JSON object";
        return false;
    }
    QJsonObject root = doc.object();

    for (const QJsonValue &v : root.value("families").toArray()) {
        QJsonObject o = v.toObject();
        BasicFamily f;
        f.id = o.value("id").toString();
        f.b64Name = o.value("b64_name").toString();
        f.indexMin = o.value("index_min").toInt(0);
        f.indexMax = o.value("index_max").toInt(99);
        if (!f.id.isEmpty()) basicFamilies_.append(f);
    }
    if (basicFamilies_.isEmpty()) {
        basicLastError_ = "basic spec has no families";
        return false;
    }

    basicSequenceDir_ = root.value("sequence_dir").toString();
    basicSequencePattern_ = root.value("sequence_pattern").toString();
    basicTablesDir_ = root.value("tables_dir").toString();
    basicProposeScript_ = root.value("propose_script").toString();
    basicNumRobotsDefault_ = root.value("num_robots_default").toInt(3);
    basicNumRobotsMin_ = root.value("num_robots_min").toInt(1);
    basicNumRobotsMax_ = root.value("num_robots_max").toInt(3);
    basicNumRobotsNote_ = root.value("num_robots_note").toString();

    for (const QJsonValue &v : root.value("refinements").toArray()) {
        QJsonObject o = v.toObject();
        BasicRefinement r;
        r.id = o.value("id").toString();
        r.name = o.value("name").toString();
        r.runner = o.value("runner").toString();
        if (o.value("extra_flags").isArray()) {
            for (const QJsonValue &fv : o.value("extra_flags").toArray())
                r.extraFlags << fv.toString();
        }
        r.model = o.value("model").toString();
        r.defaultK = o.value("default_k").toInt(40);
        r.supportsNumRobots = o.value("supports_num_robots").toBool(false);
        r.supportsVisualize = o.value("supports_visualize").toBool(false);
        if (!r.id.isEmpty()) basicRefinements_.append(r);
    }
    if (basicRefinements_.isEmpty()) {
        basicLastError_ = "basic spec has no refinements";
        return false;
    }

    return true;
}

bool LauncherWindow::allBasicRunnersKnown() const {
    for (const BasicRefinement &r : basicRefinements_) {
        if (r.runner != "cpp" && r.runner != "propose_verify") return false;
    }
    return true;
}

QStringList LauncherWindow::refinementIds() const {
    QStringList ids;
    for (const BasicRefinement &r : basicRefinements_) ids << r.id;
    return ids;
}

QString LauncherWindow::runnerForRefinement(const QString &refId) const {
    for (const BasicRefinement &r : basicRefinements_)
        if (r.id == refId) return r.runner;
    return QString();
}

QString LauncherWindow::modelForRefinement(const QString &refId) const {
    for (const BasicRefinement &r : basicRefinements_)
        if (r.id == refId) return r.model;
    return QString();
}

// ---------------------------------------------------------------------
// UI construction
// ---------------------------------------------------------------------

void LauncherWindow::setupUi() {
    setWindowTitle("MARS / ReloPush Launcher");
    resize(1100, 850);

    QWidget *central = new QWidget(this);
    setCentralWidget(central);
    QVBoxLayout *mainLayout = new QVBoxLayout(central);

    // --- Top bar (fixed, above the mode tabs): shared Display combo. ---
    // Applies to cpp-backed runs (Advanced mode, Basic mode's LNS/DQN
    // refinements); propose-and-verify refinements always force headless
    // regardless of this setting (see buildBasicCommandArgv()).
    QWidget *topBar = new QWidget(central);
    QHBoxLayout *topBarLayout = new QHBoxLayout(topBar);
    topBarLayout->setContentsMargins(0, 0, 0, 0);

    topBarLayout->addWidget(new QLabel("Display:", topBar));
    displayCombo_ = new QComboBox(topBar);
    displayCombo_->addItem("On-screen (xcb)");
    displayCombo_->addItem("Headless (offscreen)");
    displayCombo_->setToolTip(
        "On-screen (xcb): shows the Qt visualization window (needs a display).\n"
        "Headless (offscreen): no visualization window; faster, suitable for batch runs.\n"
        "Note: Basic mode's propose-and-verify (model/BC) refinements always run headless\n"
        "regardless of this setting -- they perform headless scoring only.");
    topBarLayout->addWidget(displayCombo_);
    topBarLayout->addStretch(1);

    mainLayout->addWidget(topBar);

    // --- Splitter: mode tabs (top) / output area (bottom) ---
    QSplitter *splitter = new QSplitter(Qt::Vertical, central);

    modeTabs_ = new QTabWidget(splitter);
    modeTabs_->addTab(buildBasicPage(), "Basic");
    modeTabs_->addTab(buildAdvancedPage(), "Advanced");
    splitter->addWidget(modeTabs_);

    QWidget *bottomPane = new QWidget(splitter);
    QVBoxLayout *bottomLayout = new QVBoxLayout(bottomPane);
    bottomLayout->setContentsMargins(0, 0, 0, 0);

    bottomLayout->addWidget(new QLabel("Command preview:", bottomPane));
    previewEdit_ = new QPlainTextEdit(bottomPane);
    previewEdit_->setReadOnly(true);
    previewEdit_->setMaximumBlockCount(0);
    QFont mono("Monospace");
    mono.setStyleHint(QFont::TypeWriter);
    previewEdit_->setFont(mono);
    previewEdit_->setMaximumHeight(90);
    bottomLayout->addWidget(previewEdit_);

    QWidget *buttonsRow = new QWidget(bottomPane);
    QHBoxLayout *buttonsLayout = new QHBoxLayout(buttonsRow);
    buttonsLayout->setContentsMargins(0, 0, 0, 0);
    runButton_ = new QPushButton("Run", buttonsRow);
    runButton_->setStyleSheet("QPushButton { background-color: #2e7d32; color: white; font-weight: bold; padding: 4px 12px; }");
    stopButton_ = new QPushButton("Stop", buttonsRow);
    stopButton_->setEnabled(false);
    copyButton_ = new QPushButton("Copy command", buttonsRow);
    clearButton_ = new QPushButton("Clear output", buttonsRow);
    buttonsLayout->addWidget(runButton_);
    buttonsLayout->addWidget(stopButton_);
    buttonsLayout->addWidget(copyButton_);
    buttonsLayout->addWidget(clearButton_);
    buttonsLayout->addStretch(1);
    bottomLayout->addWidget(buttonsRow);

    bottomLayout->addWidget(new QLabel("Output:", bottomPane));
    outputEdit_ = new QPlainTextEdit(bottomPane);
    outputEdit_->setReadOnly(true);
    outputEdit_->setFont(mono);
    bottomLayout->addWidget(outputEdit_, 1);

    splitter->addWidget(bottomPane);
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);

    mainLayout->addWidget(splitter, 1);

    connect(displayCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &LauncherWindow::onDisplayChanged);
    connect(modeTabs_, &QTabWidget::currentChanged, this, &LauncherWindow::onModeChanged);
    connect(runButton_, &QPushButton::clicked, this, &LauncherWindow::onRun);
    connect(stopButton_, &QPushButton::clicked, this, &LauncherWindow::onStop);
    connect(copyButton_, &QPushButton::clicked, this, &LauncherWindow::onCopyCommand);
    connect(clearButton_, &QPushButton::clicked, this, &LauncherWindow::onClearOutput);
}

QWidget *LauncherWindow::buildAdvancedPage() {
    QWidget *page = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(page);
    layout->setContentsMargins(4, 4, 4, 4);

    QWidget *exeBar = new QWidget(page);
    QHBoxLayout *exeBarLayout = new QHBoxLayout(exeBar);
    exeBarLayout->setContentsMargins(0, 0, 0, 0);
    exeBarLayout->addWidget(new QLabel("Executable:", exeBar));
    exeCombo_ = new QComboBox(exeBar);
    exeBarLayout->addWidget(exeCombo_, 1);
    layout->addWidget(exeBar);

    tabWidget_ = new QTabWidget(page);
    layout->addWidget(tabWidget_, 1);

    connect(exeCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &LauncherWindow::onExecutableChanged);

    return page;
}

QWidget *LauncherWindow::buildBasicPage() {
    QWidget *page = new QWidget();
    QVBoxLayout *layout = new QVBoxLayout(page);
    layout->setContentsMargins(8, 8, 8, 8);

    QFormLayout *form = new QFormLayout();
    form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);

    // --- Instance: Family + Index ---
    QWidget *instanceRow = new QWidget(page);
    QHBoxLayout *instanceLayout = new QHBoxLayout(instanceRow);
    instanceLayout->setContentsMargins(0, 0, 0, 0);

    basicFamilyCombo_ = new QComboBox(instanceRow);
    for (const BasicFamily &f : basicFamilies_) basicFamilyCombo_->addItem(f.id);

    basicIndexSpin_ = new QSpinBox(instanceRow);
    if (!basicFamilies_.isEmpty()) {
        basicIndexSpin_->setRange(basicFamilies_[0].indexMin, basicFamilies_[0].indexMax);
        basicIndexSpin_->setValue(basicFamilies_[0].indexMin);
    }

    instanceLayout->addWidget(new QLabel("Family:", instanceRow));
    instanceLayout->addWidget(basicFamilyCombo_);
    instanceLayout->addSpacing(16);
    instanceLayout->addWidget(new QLabel("Index:", instanceRow));
    instanceLayout->addWidget(basicIndexSpin_);
    instanceLayout->addStretch(1);
    form->addRow("Instance:", instanceRow);

    // --- Refinement ---
    basicRefinementCombo_ = new QComboBox(page);
    for (const BasicRefinement &r : basicRefinements_)
        basicRefinementCombo_->addItem(r.name, r.id);
    form->addRow("Refinement:", basicRefinementCombo_);

    // --- Number of robots ---
    basicNumRobotsSpin_ = new QSpinBox(page);
    basicNumRobotsSpin_->setRange(basicNumRobotsMin_, basicNumRobotsMax_);
    basicNumRobotsSpin_->setValue(basicNumRobotsDefault_);
    form->addRow("Number of robots:", basicNumRobotsSpin_);

    // --- Sample budget K (propose_verify only) ---
    basicKLabel_ = new QLabel("Sample budget K:");
    basicKSpin_ = new QSpinBox(page);
    basicKSpin_->setRange(1, 500);
    basicKSpin_->setValue(40);
    form->addRow(basicKLabel_, basicKSpin_);

    // --- Visualize (cpp refinements only) ---
    basicVisualizeCheck_ = new QCheckBox("Visualize", page);
    form->addRow(QString(), basicVisualizeCheck_);

    layout->addLayout(form);

    basicNoteLabel_ = new QLabel(page);
    basicNoteLabel_->setWordWrap(true);
    layout->addWidget(basicNoteLabel_);

    basicWarningLabel_ = new QLabel(page);
    basicWarningLabel_->setWordWrap(true);
    basicWarningLabel_->setStyleSheet("QLabel { color: #b71c1c; font-weight: bold; }");
    layout->addWidget(basicWarningLabel_);

    layout->addStretch(1);

    connect(basicFamilyCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &LauncherWindow::onBasicFamilyChanged);
    connect(basicIndexSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &LauncherWindow::rebuildPreview);
    connect(basicRefinementCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &LauncherWindow::onBasicRefinementChanged);
    connect(basicNumRobotsSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &LauncherWindow::rebuildPreview);
    connect(basicKSpin_, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &LauncherWindow::rebuildPreview);
    connect(basicVisualizeCheck_, &QCheckBox::toggled, this, &LauncherWindow::rebuildPreview);

    updateBasicControlsForRefinement();

    return page;
}

void LauncherWindow::populateExecutableCombo() {
    exeCombo_->clear();
    for (const ExeInfo &e : executables_) exeCombo_->addItem(e.name);
    for (int i = 0; i < executables_.size(); ++i)
        exeCombo_->setItemData(i, executables_[i].description, Qt::ToolTipRole);
    if (executables_.size() > 0) {
        exeCombo_->setCurrentIndex(0);
        exeCombo_->setToolTip(executables_[0].description);
    }
}

void LauncherWindow::rebuildForm() {
    // Destroy old tab pages (QTabWidget::clear() does not delete them).
    while (tabWidget_->count() > 0) {
        QWidget *w = tabWidget_->widget(0);
        tabWidget_->removeTab(0);
        delete w;
    }
    currentRows_.clear();

    const ExeInfo *exe = currentExecutable();
    if (!exe) {
        rebuildPreview();
        return;
    }
    const QString exeName = exe->name;

    for (const QString &group : kGroupOrder) {
        QVector<const Option *> normalOpts;
        QVector<const Option *> positionalOpts;
        for (const Option &o : allOptions_) {
            if (o.group != group) continue;
            if (!o.appliesTo.contains(exeName)) continue;
            if (o.isPositional())
                positionalOpts.append(&o);
            else
                normalOpts.append(&o);
        }
        if (normalOpts.isEmpty() && positionalOpts.isEmpty()) continue;

        std::sort(positionalOpts.begin(), positionalOpts.end(),
                  [](const Option *a, const Option *b) {
                      return a->positionalIndex < b->positionalIndex;
                  });

        QWidget *page = new QWidget();
        QVBoxLayout *pageLayout = new QVBoxLayout(page);
        pageLayout->setContentsMargins(4, 4, 4, 4);

        QScrollArea *scroll = new QScrollArea(page);
        scroll->setWidgetResizable(true);
        QWidget *formHost = new QWidget();
        QFormLayout *form = new QFormLayout(formHost);
        form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
        form->setRowWrapPolicy(QFormLayout::DontWrapRows);

        if (group == "Input" && exeName == "ReloPush-BOSS" && !positionalOpts.isEmpty()) {
            QLabel *note = new QLabel(
                "ReloPush-BOSS requires all three positional arguments together "
                "(instance file, line index, mode). They have no enable checkbox: "
                "all three are always emitted (all-or-nothing).");
            note->setWordWrap(true);
            form->addRow(note);
        }

        for (const Option *o : positionalOpts) addOptionRow(form, *o);
        for (const Option *o : normalOpts) addOptionRow(form, *o);

        scroll->setWidget(formHost);
        pageLayout->addWidget(scroll);
        tabWidget_->addTab(page, group);
    }

    rebuildPreview();
}

void LauncherWindow::addOptionRow(QFormLayout *form, const Option &opt) {
    OptionRowWidget row;
    row.opt = &opt;

    QWidget *fieldWidget = nullptr; // widget whose enabled-state follows the checkbox

    if (opt.type == "bool") {
        // No separate value editor: the checkbox itself is the toggle.
    } else if (opt.type == "enum") {
        QComboBox *combo = new QComboBox();
        combo->addItems(opt.choices);
        if (jsonValuePresent(opt.defaultValue)) {
            const QString defStr = opt.defaultValue.toVariant().toString();
            int idx = combo->findText(defStr);
            if (idx >= 0) combo->setCurrentIndex(idx);
        }
        row.combo = combo;
        fieldWidget = combo;
    } else if (opt.type == "int") {
        QSpinBox *spin = new QSpinBox();
        spin->setRange(opt.hasMin ? int(opt.min) : -1000000,
                       opt.hasMax ? int(opt.max) : 1000000000);
        int value = jsonValuePresent(opt.defaultValue)
                        ? opt.defaultValue.toVariant().toInt()
                        : spin->minimum();
        spin->setValue(value);
        row.spin = spin;
        fieldWidget = spin;
    } else if (opt.type == "float") {
        QDoubleSpinBox *dspin = new QDoubleSpinBox();
        dspin->setDecimals(6);
        dspin->setSingleStep(0.01);
        dspin->setRange(opt.hasMin ? opt.min : -1e9, opt.hasMax ? opt.max : 1e9);
        double value = jsonValuePresent(opt.defaultValue)
                            ? opt.defaultValue.toVariant().toDouble()
                            : dspin->minimum();
        dspin->setValue(value);
        row.doubleSpin = dspin;
        fieldWidget = dspin;
    } else if (opt.type == "string") {
        QLineEdit *line = new QLineEdit();
        if (jsonValuePresent(opt.defaultValue))
            line->setText(opt.defaultValue.toVariant().toString());
        row.line = line;
        fieldWidget = line;
    } else if (opt.type == "file") {
        QComboBox *combo = new QComboBox();
        combo->setEditable(true);
        combo->addItems(opt.choices);
        QString defStr = jsonValuePresent(opt.defaultValue)
                              ? opt.defaultValue.toVariant().toString()
                              : QString();
        // Required-input default: --sequence-file= starts checked and preset
        // to the first choice that actually exists on disk.
        if (opt.flag == "--sequence-file=") {
            const QString existing = firstExistingChoice(opt.choices);
            if (!existing.isEmpty()) defStr = existing;
        }
        combo->setCurrentText(defStr);
        row.combo = combo;

        QPushButton *browse = new QPushButton("Browse...");
        row.browse = browse;
        connect(browse, &QPushButton::clicked, this, [this, combo]() {
            QString start = repoRoot_;
            QString chosen = QFileDialog::getOpenFileName(this, "Select file", start);
            if (chosen.isEmpty()) return;
            QDir root(repoRoot_);
            QString rel = root.relativeFilePath(chosen);
            // Use the relative path only if it stays under repo_root (does
            // not escape via "../").
            if (!rel.startsWith("..") && !QDir::isAbsolutePath(rel))
                combo->setCurrentText(rel);
            else
                combo->setCurrentText(chosen);
        });

        QWidget *container = new QWidget();
        QHBoxLayout *hl = new QHBoxLayout(container);
        hl->setContentsMargins(0, 0, 0, 0);
        hl->addWidget(combo, 1);
        hl->addWidget(browse, 0);
        fieldWidget = container;
    }

    QString tooltip = QString("%1 — %2")
                           .arg(opt.flag.isEmpty() ? QStringLiteral("(positional argument)") : opt.flag,
                                opt.description);
    if (opt.hasMin) tooltip += QString("\nmin: %1").arg(opt.min);
    if (opt.hasMax) tooltip += QString("\nmax: %1").arg(opt.max);
    if (!opt.notes.isEmpty()) tooltip += QString("\nnotes: %1").arg(opt.notes);

    if (opt.isPositional()) {
        QLabel *label = new QLabel(opt.name + ":");
        label->setToolTip(tooltip);
        label->setWhatsThis(tooltip);
        if (fieldWidget)
            form->addRow(label, fieldWidget);
        else
            form->addRow(label);
    } else {
        QCheckBox *check = new QCheckBox(opt.name);
        check->setToolTip(tooltip);
        check->setWhatsThis(tooltip);
        // Required-input default: sequence-file starts enabled; everything
        // else starts disabled (the binary's own compiled-in default applies).
        bool defaultChecked = (opt.flag == "--sequence-file=");
        check->setChecked(defaultChecked);
        row.enableCheck = check;

        if (fieldWidget) {
            fieldWidget->setEnabled(defaultChecked);
            connect(check, &QCheckBox::toggled, fieldWidget, &QWidget::setEnabled);
            form->addRow(check, fieldWidget);
        } else {
            form->addRow(check);
        }
    }

    connectRowLiveSignals(row);
    currentRows_.append(row);
}

void LauncherWindow::connectRowLiveSignals(const OptionRowWidget &row) {
    if (row.enableCheck)
        connect(row.enableCheck, &QCheckBox::toggled, this, &LauncherWindow::rebuildPreview);
    if (row.combo)
        connect(row.combo, &QComboBox::currentTextChanged, this, &LauncherWindow::rebuildPreview);
    if (row.spin)
        connect(row.spin, QOverload<int>::of(&QSpinBox::valueChanged), this, &LauncherWindow::rebuildPreview);
    if (row.doubleSpin)
        connect(row.doubleSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
                &LauncherWindow::rebuildPreview);
    if (row.line)
        connect(row.line, &QLineEdit::textChanged, this, &LauncherWindow::rebuildPreview);
}

// ---------------------------------------------------------------------
// Helpers (shared)
// ---------------------------------------------------------------------

const ExeInfo *LauncherWindow::currentExecutable() const {
    if (!exeCombo_) return nullptr;
    const QString name = exeCombo_->currentText();
    for (const ExeInfo &e : executables_)
        if (e.name == name) return &e;
    return nullptr;
}

bool LauncherWindow::isHeadlessDisplay() const {
    return displayCombo_ && displayCombo_->currentIndex() == 1;
}

QStringList LauncherWindow::envWrapperPrefix() const {
    QString home = QString::fromLocal8Bit(qgetenv("HOME"));
    if (home.isEmpty()) home = QDir::homePath();
    const QString user = QString::fromLocal8Bit(qgetenv("USER"));

    QStringList argv;
    argv << "env" << "-i"
         << QString("HOME=%1").arg(home)
         << QString("USER=%1").arg(user)
         << "PATH=/usr/local/bin:/usr/bin:/bin";

    if (isHeadlessDisplay()) {
        argv << "QT_QPA_PLATFORM=offscreen";
    } else {
        argv << "QT_QPA_PLATFORM=xcb";
        const QString display = QString::fromLocal8Bit(qgetenv("DISPLAY"));
        if (!display.isEmpty()) argv << QString("DISPLAY=%1").arg(display);
        const QString xauthority = QString::fromLocal8Bit(qgetenv("XAUTHORITY"));
        if (!xauthority.isEmpty()) argv << QString("XAUTHORITY=%1").arg(xauthority);
    }
    return argv;
}

QString LauncherWindow::shellQuote(const QString &s) const {
    if (s.isEmpty()) return "''";
    static const QRegularExpression safe("^[A-Za-z0-9_\\-./:+,=]+$");
    if (safe.match(s).hasMatch()) return s;
    QString escaped = s;
    escaped.replace("'", "'\\''");
    return "'" + escaped + "'";
}

QString LauncherWindow::firstExistingChoice(const QStringList &choices) const {
    for (const QString &c : choices) {
        QFileInfo fi(repoRoot_ + "/" + c);
        if (fi.exists()) return c;
    }
    return QString();
}

bool LauncherWindow::currentOptionValue(const OptionRowWidget &row, QString *valueOut) const {
    const Option *o = row.opt;
    if (!o) return false;

    if (o->isPositional()) {
        if (o->type == "int" && row.spin)
            *valueOut = QString::number(row.spin->value());
        else if (row.combo)
            *valueOut = row.combo->currentText();
        else if (row.line)
            *valueOut = row.line->text();
        return true;
    }

    if (!row.enableCheck || !row.enableCheck->isChecked()) return false;

    if (o->type == "bool") {
        *valueOut = QString(); // bare flag, no value
        return true;
    }

    QString val;
    if (o->type == "int" && row.spin)
        val = QString::number(row.spin->value());
    else if (o->type == "float" && row.doubleSpin)
        val = QString::number(row.doubleSpin->value(), 'g', 10);
    else if (o->type == "string" && row.line)
        val = row.line->text();
    else if (row.combo) // enum / file
        val = row.combo->currentText();

    if (o->takesValue && val.isEmpty()) return false; // enabled but empty: skip
    *valueOut = val;
    return true;
}

// ---------------------------------------------------------------------
// Helpers (basic mode)
// ---------------------------------------------------------------------

bool LauncherWindow::isBasicMode() const {
    return !modeTabs_ || modeTabs_->currentIndex() == 0;
}

const BasicFamily *LauncherWindow::currentBasicFamily() const {
    if (!basicFamilyCombo_) return nullptr;
    const QString id = basicFamilyCombo_->currentText();
    for (const BasicFamily &f : basicFamilies_)
        if (f.id == id) return &f;
    return nullptr;
}

const BasicRefinement *LauncherWindow::currentBasicRefinement() const {
    if (!basicRefinementCombo_) return nullptr;
    const int idx = basicRefinementCombo_->currentIndex();
    if (idx < 0) return nullptr;
    const QString id = basicRefinementCombo_->itemData(idx).toString();
    for (const BasicRefinement &r : basicRefinements_)
        if (r.id == id) return &r;
    return nullptr;
}

QString LauncherWindow::basicSequenceRelPath(const BasicFamily &fam, int index) const {
    QString rel = basicSequencePattern_;
    rel.replace("{b64_name}", fam.b64Name);
    rel.replace("{index}", QString::number(index));
    return basicSequenceDir_ + "/" + rel;
}

void LauncherWindow::updateBasicControlsForRefinement() {
    const BasicRefinement *ref = currentBasicRefinement();
    if (!ref) return;

    if (basicNumRobotsSpin_) {
        basicNumRobotsSpin_->setToolTip(basicNumRobotsNote_);
        if (ref->supportsNumRobots) {
            basicNumRobotsSpin_->setEnabled(true);
        } else {
            basicNumRobotsSpin_->setValue(3);
            basicNumRobotsSpin_->setEnabled(false);
        }
    }

    const bool isProposeVerify = (ref->runner == "propose_verify");
    if (basicKSpin_) {
        basicKSpin_->setVisible(isProposeVerify);
        basicKSpin_->setEnabled(isProposeVerify);
        if (isProposeVerify) basicKSpin_->setValue(ref->defaultK);
    }
    if (basicKLabel_) basicKLabel_->setVisible(isProposeVerify);

    if (basicVisualizeCheck_) {
        basicVisualizeCheck_->setEnabled(ref->supportsVisualize);
        if (!ref->supportsVisualize) {
            basicVisualizeCheck_->setChecked(false);
            basicVisualizeCheck_->setToolTip("Visualization is not available for this refinement.");
        } else if (ref->runner == "propose_verify") {
            basicVisualizeCheck_->setToolTip(
                "After scoring, automatically opens the viewer on the best-feasible plan's "
                "saved execution (a chained second step: propose with --result-out=, then "
                "--play-result= -- no re-run of the search).");
        } else {
            basicVisualizeCheck_->setToolTip(QString());
        }
    }

    if (basicNoteLabel_) {
        if (ref->runner == "cpp") {
            basicNoteLabel_->setText(
                QString("Runs phastar_push_demo directly (%1).").arg(ref->extraFlags.join(' ')));
        } else {
            basicNoteLabel_->setText(
                "Runs propose_best_plan.py (Python drives the C++ oracle internally to score "
                "sampled plans; ~10s at k=40). Always runs headless regardless of the Display "
                "setting above.");
        }
    }

    if (basicRefinementCombo_)
        basicRefinementCombo_->setToolTip(QString("runner: %1").arg(ref->runner));
}

QString LauncherWindow::basicExistenceWarning() const {
    const BasicRefinement *ref = currentBasicRefinement();
    const BasicFamily *fam = currentBasicFamily();
    if (!ref || !fam) return QString();

    const int index = basicIndexSpin_ ? basicIndexSpin_->value() : 0;
    QStringList missing;

    const QString seqRel = basicSequenceRelPath(*fam, index);
    if (!QFileInfo::exists(repoRoot_ + "/" + seqRel)) missing << seqRel;

    if (ref->runner == "propose_verify") {
        const QString tableRel =
            basicTablesDir_ + QString("/%1_ind%2.json").arg(fam->id).arg(index);
        if (!QFileInfo::exists(repoRoot_ + "/" + tableRel)) missing << tableRel;
        if (!ref->model.isEmpty() && !QFileInfo::exists(repoRoot_ + "/" + ref->model))
            missing << ref->model;
    }

    if (missing.isEmpty()) return QString();
    return "Warning: missing file(s) - " + missing.join(", ");
}

// ---------------------------------------------------------------------
// Command construction
// ---------------------------------------------------------------------

QStringList LauncherWindow::buildCommandArgv() const {
    return isBasicMode() ? buildBasicCommandArgv() : buildAdvancedCommandArgv();
}

QStringList LauncherWindow::buildAdvancedCommandArgv() const {
    QStringList argv = envWrapperPrefix();

    const ExeInfo *exe = currentExecutable();
    if (!exe) return argv;
    argv << exe->path;

    // Positional args (ReloPush-BOSS only), in positional_index order, before
    // any flags.
    QVector<const OptionRowWidget *> positionalRows;
    for (const OptionRowWidget &row : currentRows_)
        if (row.opt && row.opt->isPositional()) positionalRows.append(&row);
    std::sort(positionalRows.begin(), positionalRows.end(),
              [](const OptionRowWidget *a, const OptionRowWidget *b) {
                  return a->opt->positionalIndex < b->opt->positionalIndex;
              });
    for (const OptionRowWidget *row : positionalRows) {
        QString val;
        if (currentOptionValue(*row, &val)) argv << val;
    }

    // Flags for every enabled option.
    for (const OptionRowWidget &row : currentRows_) {
        if (!row.opt || row.opt->isPositional()) continue;
        QString val;
        if (!currentOptionValue(row, &val)) continue;
        const Option *o = row.opt;
        if (o->type == "bool" || !o->takesValue) {
            argv << o->flag;
        } else if (o->valueSeparator == "=") {
            argv << (o->flag + val);
        } else { // space-separated
            QString flagToken = o->flag;
            if (flagToken.endsWith(' ')) flagToken.chop(1);
            argv << flagToken << val;
        }
    }

    return argv;
}

QStringList LauncherWindow::buildBasicCommandArgv() const {
    QStringList argv;

    const BasicRefinement *ref = currentBasicRefinement();
    const BasicFamily *fam = currentBasicFamily();
    if (!ref || !fam) return argv;

    const int index = basicIndexSpin_ ? basicIndexSpin_->value() : 0;

    QString home = QString::fromLocal8Bit(qgetenv("HOME"));
    if (home.isEmpty()) home = QDir::homePath();
    const QString user = QString::fromLocal8Bit(qgetenv("USER"));

    if (ref->runner == "cpp") {
        argv << envWrapperPrefix();

        QString exePath;
        for (const ExeInfo &e : executables_)
            if (e.name == "phastar_push_demo") exePath = e.path;
        if (exePath.isEmpty()) return QStringList();
        argv << exePath;

        argv << QString("--sequence-file=%1").arg(basicSequenceRelPath(*fam, index));

        const int numRobots =
            basicNumRobotsSpin_ ? basicNumRobotsSpin_->value() : basicNumRobotsDefault_;
        argv << QString("--num-robots=%1").arg(numRobots);

        for (const QString &flag : ref->extraFlags) argv << flag;

        if (ref->supportsVisualize && basicVisualizeCheck_ && basicVisualizeCheck_->isChecked())
            argv << "--visualize";
    } else if (ref->runner == "propose_verify") {
        // Always offscreen: this is headless scoring, independent of the
        // shared Display combo (which only governs the visualization window
        // of a cpp-backed run).
        argv << "env" << "-i"
             << QString("HOME=%1").arg(home)
             << QString("USER=%1").arg(user)
             << "PATH=/usr/local/bin:/usr/bin:/bin"
             << "QT_QPA_PLATFORM=offscreen";

        argv << "python3" << basicProposeScript_;
        argv << "--family" << fam->id;
        argv << "--index" << QString::number(index);
        argv << "--model" << ref->model;

        const int k = basicKSpin_ ? basicKSpin_->value() : ref->defaultK;
        argv << "--k" << QString::number(k);
        argv << "--seed" << "1";

        const QString outBest =
            QDir(basicTmpDir()).filePath(QString("best_%1_ind%2.jsonl").arg(fam->id).arg(index));
        argv << "--out-best" << outBest;

        // Visualize (propose_verify): thread --result-out through to the
        // oracle call so the best-feasible plan's ExecutedScenario is saved;
        // onRun() chains a `--play-result=` follow-up step onto this same
        // path once this process exits 0. Unchecked: no flag, no chain --
        // unchanged from before this feature existed.
        if (ref->supportsVisualize && basicVisualizeCheck_ && basicVisualizeCheck_->isChecked())
            argv << "--result-out" << basicResultPath();
    }

    return argv;
}

QString LauncherWindow::basicTmpDir() const {
    QString tmpDir = "/home/jeeho/.claude/jobs/e54e1d13/tmp";
    if (!QDir(tmpDir).exists()) tmpDir = QDir::tempPath();
    return tmpDir;
}

QString LauncherWindow::basicResultPath() const {
    const BasicFamily *fam = currentBasicFamily();
    if (!fam) return QString();
    const int index = basicIndexSpin_ ? basicIndexSpin_->value() : 0;
    return QDir(basicTmpDir()).filePath(
        QString("mars_launcher_result_%1_ind%2.scn.b64").arg(fam->id).arg(index));
}

QStringList LauncherWindow::buildPlayResultArgv(const QString &resultPath) const {
    if (resultPath.isEmpty()) return QStringList();
    QStringList argv = envWrapperPrefix();
    QString exePath;
    for (const ExeInfo &e : executables_)
        if (e.name == "phastar_push_demo") exePath = e.path;
    if (exePath.isEmpty()) return QStringList();
    argv << exePath;
    argv << QString("--play-result=%1").arg(resultPath);
    return argv;
}

QStringList LauncherWindow::buildBasicFollowupArgv() const {
    if (!isBasicMode()) return QStringList();
    const BasicRefinement *ref = currentBasicRefinement();
    if (!ref || ref->runner != "propose_verify" || !ref->supportsVisualize) return QStringList();
    if (!basicVisualizeCheck_ || !basicVisualizeCheck_->isChecked()) return QStringList();
    return buildPlayResultArgv(basicResultPath());
}

QString LauncherWindow::buildPreviewLine() const {
    const QStringList argv = buildCommandArgv();
    QStringList quoted;
    quoted.reserve(argv.size());
    for (const QString &tok : argv) quoted << shellQuote(tok);
    const QString cwd = envCwd_.isEmpty() ? repoRoot_ : envCwd_;
    QString line = QString("cd %1 && %2").arg(shellQuote(cwd), quoted.join(' '));

    const QStringList followupArgv = buildBasicFollowupArgv();
    if (!followupArgv.isEmpty()) {
        QStringList followupQuoted;
        followupQuoted.reserve(followupArgv.size());
        for (const QString &tok : followupArgv) followupQuoted << shellQuote(tok);
        const QString followupLine =
            QString("cd %1 && %2").arg(shellQuote(cwd), followupQuoted.join(' '));
        line += "\n# then, on success:\n" + followupLine;
    }
    return line;
}

// ---------------------------------------------------------------------
// Public API used by the top bar / self-test: Advanced mode
// ---------------------------------------------------------------------

bool LauncherWindow::selectExecutable(const QString &exeName) {
    int idx = exeCombo_->findText(exeName);
    if (idx < 0) return false;
    exeCombo_->setCurrentIndex(idx); // triggers onExecutableChanged -> rebuildForm
    rebuildForm();                   // idempotent; ensures form is fresh even if index was unchanged
    return true;
}

// ---------------------------------------------------------------------
// Public API used by self-test: Basic mode
// ---------------------------------------------------------------------

void LauncherWindow::switchToBasicMode() {
    if (modeTabs_) modeTabs_->setCurrentIndex(0);
}

void LauncherWindow::switchToAdvancedMode() {
    if (modeTabs_) modeTabs_->setCurrentIndex(1);
}

bool LauncherWindow::selectBasicFamily(const QString &familyId) {
    if (!basicFamilyCombo_) return false;
    int idx = basicFamilyCombo_->findText(familyId);
    if (idx < 0) return false;
    basicFamilyCombo_->setCurrentIndex(idx);
    onBasicFamilyChanged(idx); // idempotent; ensures the index range refreshes
    return true;
}

void LauncherWindow::setBasicIndex(int index) {
    if (basicIndexSpin_) basicIndexSpin_->setValue(index);
}

void LauncherWindow::setBasicNumRobots(int n) {
    if (basicNumRobotsSpin_) basicNumRobotsSpin_->setValue(n);
}

bool LauncherWindow::selectBasicRefinement(const QString &refId) {
    if (!basicRefinementCombo_) return false;
    for (int i = 0; i < basicRefinementCombo_->count(); ++i) {
        if (basicRefinementCombo_->itemData(i).toString() == refId) {
            basicRefinementCombo_->setCurrentIndex(i);
            updateBasicControlsForRefinement(); // idempotent
            return true;
        }
    }
    return false;
}

void LauncherWindow::setBasicK(int k) {
    if (basicKSpin_) basicKSpin_->setValue(k);
}

void LauncherWindow::setBasicVisualize(bool on) {
    if (basicVisualizeCheck_) basicVisualizeCheck_->setChecked(on);
}

// ---------------------------------------------------------------------
// Slots
// ---------------------------------------------------------------------

void LauncherWindow::onExecutableChanged(int index) {
    Q_UNUSED(index);
    const ExeInfo *exe = currentExecutable();
    if (exe) exeCombo_->setToolTip(exe->description);
    rebuildForm();
}

void LauncherWindow::onDisplayChanged(int index) {
    Q_UNUSED(index);
    rebuildPreview();
}

void LauncherWindow::onModeChanged(int index) {
    Q_UNUSED(index);
    rebuildPreview();
}

void LauncherWindow::onBasicFamilyChanged(int index) {
    Q_UNUSED(index);
    const BasicFamily *fam = currentBasicFamily();
    if (fam && basicIndexSpin_) {
        const int oldValue = basicIndexSpin_->value();
        basicIndexSpin_->setRange(fam->indexMin, fam->indexMax);
        basicIndexSpin_->setValue(qBound(fam->indexMin, oldValue, fam->indexMax));
    }
    rebuildPreview();
}

void LauncherWindow::onBasicRefinementChanged(int index) {
    Q_UNUSED(index);
    updateBasicControlsForRefinement();
    rebuildPreview();
}

void LauncherWindow::rebuildPreview() {
    if (previewEdit_) previewEdit_->setPlainText(buildPreviewLine());
    if (basicWarningLabel_)
        basicWarningLabel_->setText(isBasicMode() ? basicExistenceWarning() : QString());
}

void LauncherWindow::onRun() {
    if (process_ && process_->state() != QProcess::NotRunning) return;

    QStringList argv = buildCommandArgv();
    if (argv.isEmpty()) return;

    if (isBasicMode() && outputEdit_) {
        const QString warning = basicExistenceWarning();
        if (!warning.isEmpty()) outputEdit_->appendPlainText(warning);
    }

    // Queue the Step B (--play-result=) follow-up now, before Step A starts,
    // so onProcessFinished() knows to chain it once this run exits 0. Empty
    // for every case except propose_verify+Visualize-checked (cpp runs,
    // Visualize-unchecked propose_verify runs: no chain, same as before this
    // feature existed).
    pendingFollowupArgv_ = buildBasicFollowupArgv();
    pendingFollowupResultPath_ = pendingFollowupArgv_.isEmpty() ? QString() : basicResultPath();
    pendingFollowupPreview_.clear();
    if (!pendingFollowupArgv_.isEmpty()) {
        QStringList quoted;
        quoted.reserve(pendingFollowupArgv_.size());
        for (const QString &tok : pendingFollowupArgv_) quoted << shellQuote(tok);
        const QString cwd = envCwd_.isEmpty() ? repoRoot_ : envCwd_;
        pendingFollowupPreview_ = QString("cd %1 && %2").arg(shellQuote(cwd), quoted.join(' '));
    }

    process_ = new QProcess(this);
    connect(process_, &QProcess::readyRead, this, &LauncherWindow::onProcessReadyRead);
    connect(process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &LauncherWindow::onProcessFinished);
    connect(process_, &QProcess::errorOccurred, this, &LauncherWindow::onProcessErrorOccurred);

    process_->setWorkingDirectory(envCwd_.isEmpty() ? repoRoot_ : envCwd_);
    process_->setProcessChannelMode(QProcess::MergedChannels);

    outputEdit_->appendPlainText("$ " + buildPreviewLine());

    QStringList args = argv;
    args.removeFirst(); // drop the leading "env" program name
    process_->start("env", args);

    runButton_->setEnabled(false);
    stopButton_->setEnabled(true);
}

void LauncherWindow::onStop() {
    if (!process_) return;
    // Clear any queued Step B so a user abort (of either step) never lets
    // the follow-up fire once the terminated/killed process's `finished`
    // signal arrives.
    pendingFollowupArgv_.clear();
    pendingFollowupPreview_.clear();
    pendingFollowupResultPath_.clear();
    process_->terminate();
    QTimer::singleShot(2000, this, [this]() {
        if (process_ && process_->state() != QProcess::NotRunning) process_->kill();
    });
}

void LauncherWindow::onCopyCommand() {
    QApplication::clipboard()->setText(buildPreviewLine());
}

void LauncherWindow::onClearOutput() {
    if (outputEdit_) outputEdit_->clear();
}

void LauncherWindow::onProcessReadyRead() {
    if (!process_ || !outputEdit_) return;
    const QString text = QString::fromLocal8Bit(process_->readAll());
    QTextCursor cursor = outputEdit_->textCursor();
    cursor.movePosition(QTextCursor::End);
    outputEdit_->setTextCursor(cursor);
    outputEdit_->insertPlainText(text);
    outputEdit_->ensureCursorVisible();
}

void LauncherWindow::onProcessFinished(int exitCode, QProcess::ExitStatus status) {
    const QString statusStr = (status == QProcess::CrashExit) ? "crashed" : "exited";
    outputEdit_->appendPlainText(
        QString("\n=== finished, exit code %1 (%2) ===").arg(exitCode).arg(statusStr));
    if (process_) {
        process_->deleteLater();
        process_ = nullptr;
    }

    // Step A -> Step B chain: if a follow-up (--play-result=) is queued,
    // this "finished" is Step A's. Consume (clear) it unconditionally so it
    // can never fire twice, then only actually start it if Step A succeeded
    // and really produced the result file.
    if (!pendingFollowupArgv_.isEmpty()) {
        const QStringList followupArgv = pendingFollowupArgv_;
        const QString followupPreview = pendingFollowupPreview_;
        const QString resultPath = pendingFollowupResultPath_;
        pendingFollowupArgv_.clear();
        pendingFollowupPreview_.clear();
        pendingFollowupResultPath_.clear();

        if (exitCode == 0 && !resultPath.isEmpty() && QFileInfo::exists(resultPath)) {
            outputEdit_->appendPlainText("$ " + followupPreview);

            process_ = new QProcess(this);
            connect(process_, &QProcess::readyRead, this, &LauncherWindow::onProcessReadyRead);
            connect(process_, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                    this, &LauncherWindow::onProcessFinished);
            connect(process_, &QProcess::errorOccurred, this, &LauncherWindow::onProcessErrorOccurred);
            process_->setWorkingDirectory(envCwd_.isEmpty() ? repoRoot_ : envCwd_);
            process_->setProcessChannelMode(QProcess::MergedChannels);

            QStringList args = followupArgv;
            args.removeFirst(); // drop the leading "env" program name
            process_->start("env", args);

            runButton_->setEnabled(false);
            stopButton_->setEnabled(true);
            return; // Step B is now running; Run/Stop state stays "running"
        }

        outputEdit_->appendPlainText("[visualize] skipped: no result file / propose failed");
    }

    runButton_->setEnabled(true);
    stopButton_->setEnabled(false);
}

void LauncherWindow::onProcessErrorOccurred(QProcess::ProcessError error) {
    Q_UNUSED(error);
    if (process_)
        outputEdit_->appendPlainText(QString("\n=== process error: %1 ===").arg(process_->errorString()));
    // A process that failed to start (e.g. FailedToStart) never emits
    // `finished`, so any queued follow-up must be cleared here too -- else
    // it would wrongly fire on a later, unrelated run's `finished` signal.
    pendingFollowupArgv_.clear();
    pendingFollowupPreview_.clear();
    pendingFollowupResultPath_.clear();
    runButton_->setEnabled(true);
    stopButton_->setEnabled(false);
}
