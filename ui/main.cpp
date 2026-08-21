// main.cpp - entry point for mars_launcher.
//
// Normal mode: shows the LauncherWindow.
// `--selftest`: headless verification (no window shown) - see runSelfTest()
// below. Intended to run under QT_QPA_PLATFORM=offscreen.

#include "LauncherWindow.h"

#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QMessageBox>
#include <QString>
#include <QStringList>

#include <cstdio>

namespace {

// Spec path resolution order (see project brief):
//   1. --spec=<path> argv
//   2. DEFAULT_SPEC_PATH compile definition, if that file exists
//   3. options_spec.json next to the running executable
//   4. ./ui/options_spec.json relative to the current working directory
QString resolveSpecPath(const QStringList &args) {
    for (const QString &a : args) {
        if (a.startsWith("--spec=")) return a.mid(QStringLiteral("--spec=").size());
    }

#ifdef DEFAULT_SPEC_PATH
    {
        const QString defaultPath = QStringLiteral(DEFAULT_SPEC_PATH);
        if (QFileInfo::exists(defaultPath)) return defaultPath;
    }
#endif

    const QString besideExe =
        QFileInfo(QCoreApplication::applicationDirPath(), "options_spec.json").absoluteFilePath();
    if (QFileInfo::exists(besideExe)) return besideExe;

    const QString relToCwd = QDir::current().filePath("ui/options_spec.json");
    if (QFileInfo::exists(relToCwd)) return relToCwd;

    return QString();
}

// Same resolution strategy as resolveSpecPath(), for basic_refinements.json:
//   1. --basic-spec=<path> argv
//   2. DEFAULT_BASIC_REFINEMENTS_PATH compile definition, if that file exists
//   3. basic_refinements.json next to the running executable
//   4. ./ui/basic_refinements.json relative to the current working directory
// Returning an empty string is not fatal: LauncherWindow falls back to a
// file named "basic_refinements.json" beside the resolved spec path.
QString resolveBasicRefinementsPath(const QStringList &args) {
    for (const QString &a : args) {
        if (a.startsWith("--basic-spec=")) return a.mid(QStringLiteral("--basic-spec=").size());
    }

#ifdef DEFAULT_BASIC_REFINEMENTS_PATH
    {
        const QString defaultPath = QStringLiteral(DEFAULT_BASIC_REFINEMENTS_PATH);
        if (QFileInfo::exists(defaultPath)) return defaultPath;
    }
#endif

    const QString besideExe =
        QFileInfo(QCoreApplication::applicationDirPath(), "basic_refinements.json").absoluteFilePath();
    if (QFileInfo::exists(besideExe)) return besideExe;

    const QString relToCwd = QDir::current().filePath("ui/basic_refinements.json");
    if (QFileInfo::exists(relToCwd)) return relToCwd;

    return QString();
}

#define SELFTEST_ASSERT(cond, msg)                                              \
    do {                                                                        \
        if (!(cond)) {                                                         \
            std::fprintf(stderr, "SELFTEST FAILED: %s\n", (msg));               \
            return 1;                                                          \
        }                                                                       \
    } while (0)

int runSelfTest(const QString &specPath, const QString &basicSpecPath) {
    LauncherWindow win(specPath, basicSpecPath);

    SELFTEST_ASSERT(win.specLoadOk(),
                     qPrintable(QString("spec failed to load: %1").arg(win.lastError())));

    SELFTEST_ASSERT(win.allOptionTypesKnown(), "an option has an unrecognized 'type'");

    const QStringList exeNames = win.executableNames();
    SELFTEST_ASSERT(!exeNames.isEmpty(), "no executables found in spec");

    win.switchToAdvancedMode();

    QStringList demoArgv;
    QStringList bossArgv;

    for (const QString &name : exeNames) {
        SELFTEST_ASSERT(win.selectExecutable(name),
                        qPrintable(QString("could not select executable '%1'").arg(name)));
        const QStringList argv = win.buildCommandArgv();
        const QString preview = win.buildPreviewLine();
        std::printf("[%s] %s\n", qPrintable(name), qPrintable(preview));

        if (name == "phastar_push_demo") demoArgv = argv;
        if (name == "ReloPush-BOSS") bossArgv = argv;
    }

    // --- Assertion 4: phastar_push_demo default command ---
    SELFTEST_ASSERT(!demoArgv.isEmpty(), "phastar_push_demo argv is empty");
    SELFTEST_ASSERT(demoArgv.size() >= 2 && demoArgv.at(0) == "env" && demoArgv.at(1) == "-i",
                     "phastar_push_demo argv does not start with 'env -i'");

    bool hasExePath = false;
    bool hasSequenceFile = false;
    QString sequenceFileArg;
    bool hasEmptyToken = false;
    for (const QString &tok : demoArgv) {
        if (tok.isEmpty()) hasEmptyToken = true;
        if (tok.endsWith("/phastar_push_demo")) hasExePath = true;
        if (tok.startsWith("--sequence-file=")) {
            hasSequenceFile = true;
            sequenceFileArg = tok.mid(QStringLiteral("--sequence-file=").size());
        }
    }
    SELFTEST_ASSERT(hasExePath, "phastar_push_demo argv missing the executable path");
    SELFTEST_ASSERT(!hasEmptyToken, "phastar_push_demo argv contains an empty token");
    SELFTEST_ASSERT(hasSequenceFile, "phastar_push_demo argv missing a --sequence-file= token");
    SELFTEST_ASSERT(sequenceFileArg.endsWith(".b64"), "default --sequence-file= value is not a .b64 path");
    {
        const QString absSeqPath = win.repoRoot() + "/" + sequenceFileArg;
        SELFTEST_ASSERT(QFileInfo::exists(absSeqPath),
                        qPrintable(QString("default --sequence-file= target does not exist on disk: %1")
                                       .arg(absSeqPath)));
    }

    // --- Assertion 5: ReloPush-BOSS positional args ---
    SELFTEST_ASSERT(!bossArgv.isEmpty(), "ReloPush-BOSS argv is empty");
    int exeIdx = -1;
    for (int i = 0; i < bossArgv.size(); ++i) {
        if (bossArgv.at(i).endsWith("/ReloPush-BOSS")) {
            exeIdx = i;
            break;
        }
    }
    SELFTEST_ASSERT(exeIdx >= 0, "ReloPush-BOSS argv missing the executable path");

    int firstFlagIdx = bossArgv.size();
    for (int i = exeIdx + 1; i < bossArgv.size(); ++i) {
        if (bossArgv.at(i).startsWith("--")) {
            firstFlagIdx = i;
            break;
        }
    }
    const int positionalCount = firstFlagIdx - (exeIdx + 1);
    SELFTEST_ASSERT(positionalCount == 3,
                    qPrintable(QString("expected exactly 3 positional args before the first "
                                        "flag for ReloPush-BOSS, got %1")
                                   .arg(positionalCount)));

    bool anyEmptyBoss = false;
    for (const QString &tok : bossArgv)
        if (tok.isEmpty()) anyEmptyBoss = true;
    SELFTEST_ASSERT(!anyEmptyBoss, "ReloPush-BOSS argv contains an empty token");

    // =====================================================================
    // Basic mode
    // =====================================================================

    SELFTEST_ASSERT(win.basicSpecLoadOk(),
                     qPrintable(QString("basic_refinements.json failed to load: %1")
                                    .arg(win.basicLastError())));
    SELFTEST_ASSERT(win.allBasicRunnersKnown(),
                     "a basic refinement has an unrecognized 'runner' (expected cpp|propose_verify)");

    const QStringList refIds = win.refinementIds();
    SELFTEST_ASSERT(!refIds.isEmpty(), "no refinements found in basic_refinements.json");

    win.switchToBasicMode();
    SELFTEST_ASSERT(win.selectBasicFamily("boss8"), "could not select basic family 'boss8'");

    for (const QString &refId : refIds) {
        SELFTEST_ASSERT(win.selectBasicRefinement(refId),
                        qPrintable(QString("could not select basic refinement '%1'").arg(refId)));
        // Re-apply the shared instance/robot-count/K fields every iteration:
        // selecting a refinement can reset num-robots (propose_verify forces
        // it to 3) and K (defaults to that refinement's default_k).
        win.setBasicIndex(3);
        win.setBasicNumRobots(3);
        win.setBasicK(40);

        const QStringList argv = win.buildCommandArgv();
        const QString preview = win.buildPreviewLine();
        std::printf("[basic:%s] %s\n", qPrintable(refId), qPrintable(preview));

        bool hasEmptyBasicToken = false;
        for (const QString &tok : argv)
            if (tok.isEmpty()) hasEmptyBasicToken = true;
        SELFTEST_ASSERT(!hasEmptyBasicToken,
                        qPrintable(QString("basic refinement '%1' argv contains an empty token")
                                       .arg(refId)));

        const QString runner = win.runnerForRefinement(refId);
        const QString joined = argv.join(' ');

        if (runner == "cpp") {
            bool hasBasicExePath = false;
            for (const QString &tok : argv)
                if (tok.endsWith("/phastar_push_demo")) hasBasicExePath = true;
            SELFTEST_ASSERT(hasBasicExePath,
                            qPrintable(QString("basic refinement '%1' argv missing phastar_push_demo path")
                                           .arg(refId)));

            SELFTEST_ASSERT(
                joined.contains("--sequence-file="
                                 "results/relopush-out/result_seq_ReloPush-BOSS_8_objects.txt_ind3.b64"),
                qPrintable(QString("basic refinement '%1' argv missing the expected --sequence-file=")
                               .arg(refId)));

            bool hasSearchModeFlag = false;
            for (const QString &tok : argv)
                if (tok.startsWith("--search-mode=")) hasSearchModeFlag = true;
            SELFTEST_ASSERT(hasSearchModeFlag,
                            qPrintable(QString("basic refinement '%1' argv missing --search-mode=")
                                           .arg(refId)));
        } else if (runner == "propose_verify") {
            SELFTEST_ASSERT(argv.contains("python3"),
                            qPrintable(QString("basic refinement '%1' argv missing python3").arg(refId)));

            bool hasProposeScript = false;
            for (const QString &tok : argv)
                if (tok.contains("propose_best_plan.py")) hasProposeScript = true;
            SELFTEST_ASSERT(hasProposeScript,
                            qPrintable(QString("basic refinement '%1' argv missing propose_best_plan.py")
                                           .arg(refId)));

            SELFTEST_ASSERT(joined.contains("--family boss8"),
                            qPrintable(QString("basic refinement '%1' argv missing --family boss8")
                                           .arg(refId)));
            SELFTEST_ASSERT(joined.contains("--index 3"),
                            qPrintable(QString("basic refinement '%1' argv missing --index 3")
                                           .arg(refId)));

            const QString model = win.modelForRefinement(refId);
            SELFTEST_ASSERT(!model.isEmpty(),
                            qPrintable(QString("basic refinement '%1' has no model path").arg(refId)));
            SELFTEST_ASSERT(joined.contains(QString("--model %1").arg(model)),
                            qPrintable(QString("basic refinement '%1' argv missing --model %2")
                                           .arg(refId, model)));

            SELFTEST_ASSERT(joined.contains("--k 40"),
                            qPrintable(QString("basic refinement '%1' argv missing --k 40").arg(refId)));
            SELFTEST_ASSERT(argv.contains("--out-best"),
                            qPrintable(QString("basic refinement '%1' argv missing --out-best")
                                           .arg(refId)));
        } else {
            SELFTEST_ASSERT(false,
                            qPrintable(QString("basic refinement '%1' has unknown runner '%2'")
                                           .arg(refId, runner)));
        }
    }

    // =====================================================================
    // Visualize chain (propose_verify + Visualize ON): the "save-and-replay"
    // follow-up wired onto propose_best_plan.py's --result-out= /
    // phastar_push_demo's --play-result=. Step A's argv must carry
    // --result-out <path>.scn.b64; a separate, non-running-process query
    // (buildBasicFollowupArgv()) must report the Step B argv
    // (--play-result=<path>.scn.b64 + the phastar_push_demo path) that would
    // be chained on success.
    // =====================================================================

    SELFTEST_ASSERT(win.selectBasicRefinement("pg_ep7"),
                    "could not select basic refinement 'pg_ep7' for the visualize-chain check");
    win.setBasicIndex(3);
    win.setBasicNumRobots(3);
    win.setBasicK(40);
    win.setBasicVisualize(true);

    const QStringList visArgv = win.buildCommandArgv();
    const QString visJoined = visArgv.join(' ');
    const QString visPreview = win.buildPreviewLine();
    std::printf("[basic:pg_ep7+visualize] %s\n", qPrintable(visPreview));

    bool hasResultOutFlag = false;
    for (const QString &tok : visArgv)
        if (tok == "--result-out") hasResultOutFlag = true;
    SELFTEST_ASSERT(hasResultOutFlag,
                    "propose_verify+Visualize Step-A argv missing --result-out");
    SELFTEST_ASSERT(visJoined.contains(".scn.b64"),
                    "propose_verify+Visualize --result-out value is not a .scn.b64 path");
    SELFTEST_ASSERT(visPreview.contains("# then, on success:"),
                    "propose_verify+Visualize preview missing the chained Step-B line");

    const QStringList followupArgv = win.buildBasicFollowupArgv();
    SELFTEST_ASSERT(!followupArgv.isEmpty(),
                    "propose_verify+Visualize produced no follow-up play argv");
    bool hasPlayResultFlag = false;
    bool hasFollowupExePath = false;
    for (const QString &tok : followupArgv) {
        if (tok.startsWith("--play-result=") && tok.endsWith(".scn.b64")) hasPlayResultFlag = true;
        if (tok.endsWith("/phastar_push_demo")) hasFollowupExePath = true;
    }
    SELFTEST_ASSERT(hasPlayResultFlag,
                    "follow-up argv missing --play-result=...scn.b64");
    SELFTEST_ASSERT(hasFollowupExePath,
                    "follow-up argv missing the phastar_push_demo path");

    // Visualize OFF: no --result-out, no chain -- unchanged from before this
    // feature existed.
    win.setBasicVisualize(false);
    const QStringList noVisArgv = win.buildCommandArgv();
    for (const QString &tok : noVisArgv)
        SELFTEST_ASSERT(tok != "--result-out",
                        "propose_verify with Visualize OFF must not include --result-out");
    SELFTEST_ASSERT(win.buildBasicFollowupArgv().isEmpty(),
                    "propose_verify with Visualize OFF must produce no follow-up argv");
    SELFTEST_ASSERT(!win.buildPreviewLine().contains("# then, on success:"),
                    "propose_verify with Visualize OFF must not show a chained preview line");

    std::printf("SELFTEST_OK\n");
    return 0;
}

} // namespace

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    QStringList args;
    for (int i = 1; i < argc; ++i) args << QString::fromLocal8Bit(argv[i]);

    const bool selftest = args.contains("--selftest");

    const QString specPath = resolveSpecPath(args);
    const QString basicSpecPath = resolveBasicRefinementsPath(args);
    if (specPath.isEmpty()) {
        const QString msg = "Could not locate options_spec.json (checked --spec=, "
                             "DEFAULT_SPEC_PATH, next to the executable, and ./ui/options_spec.json).";
        if (selftest) {
            std::fprintf(stderr, "SELFTEST FAILED: %s\n", qPrintable(msg));
            return 1;
        }
        QMessageBox::critical(nullptr, "mars_launcher", msg);
        return 1;
    }

    if (selftest) {
        return runSelfTest(specPath, basicSpecPath);
    }

    LauncherWindow window(specPath, basicSpecPath);
    window.show();
    return app.exec();
}
