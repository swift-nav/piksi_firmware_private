#!groovy

// Use 'ci-jenkins@someref' to pull shared lib from a different branch/tag than the default.
// Default is the master branch
@Library("ci-jenkins") import com.swiftnav.ci.*

def context = new Context(context: this)
context.setRepo("piksi_firmware_private")

def builder = context.getBuilder()
def logger = context.getLogger()
def hitl = new SwiftHitl(context: context)
hitl.update()

String dockerMountArgs = "-v /mnt/efs/refrepo:/mnt/efs/refrepo"

pipeline {
    // Override agent in each stage to make sure we don't share containers among stages.
    agent any
    options {
        // Make sure job aborts eventually if hanging.
        timeout(time: 2, unit: 'HOURS')
        timestamps()
        // Keep builds for 7 days.
        buildDiscarder(logRotator(daysToKeepStr: '7'))
    }

    parameters {
        choice(name: "LOG_LEVEL", choices: ['info', 'debug', 'warning', 'error'])
        booleanParam(name: "FORCE_ARTIFACTS_PUSH", defaultValue: false)
    }

    stages {
        stage('Build') {
            parallel {
                stage('Build Firmware') {
                    agent {
                        dockerfile {
                            args dockerMountArgs
                        }
                    }
                    environment {
                        PIKSI_HW = "v3"
                        HITL_API_BUILD_TYPE = "pull_request"
                        HITL_VIEWER_BUILD_TYPE = "pr"
                        PRODUCT_VERSION = "v3"
                        SLACK_CHANNEL = "github"
                    }
                    steps {
                        stageStart()
                        gitPrep()

                        script {
                            runMake(target: "PIKSI_REV=prod all", workDir: ".")
                            runMake(target: "PIKSI_REV=base all", workDir: ".")

                            if (context.isPrPush()) {
                                createPrDescription(context: context)
                            }
                            context.archivePatterns(patterns: [
                                "pr_description.yaml",
                                "requirements.yaml"])
                            if (!context.isTagPush() || context.pipe.params.FORCE_ARTIFACTS_PUSH) {
                                context.archivePatterns(patterns: [
                                    "build_v3_prod/piksi_firmware_v3_prod*.elf",
                                    "build_v3_prod/piksi_firmware_v3_prod.map",
                                    "build_v3_base/piksi_firmware_v3_base*.elf",
                                    "build_v3_base/piksi_firmware_v3_base.map"],
                                    addPath: "v3")
                            }
                            if (false) { // Disable until hitl issue is fixed
                                hitl.triggerForPr() // this generates metrics.yaml
                                context.archivePatterns(
                                    patterns: [
                                        'metrics.yaml',
                                    ],
                                )
                            }
                            hitl.addComments()
                        }
                    }
                }
                stage('Tests & Mesta') {
                    agent {
                        dockerfile {
                            args dockerMountArgs
                        }
                    }
                    environment {
                        PIKSI_HW = "v3"
                    }
                    steps {
                        stageStart()
                        gitPrep()

                        script {
                            runMake(target: "starling-cmake", workDir: ".")
                            runMake(target: "run_tests", workDir: ".")
                            runMake(target: "mesta", workDir: "mesta")
                            sh script: "./mesta/mesta"
                        }
                    }
                }
                stage('Formatting & Lint') {
                    agent {
                        dockerfile {
                            args dockerMountArgs
                        }
                    }
                    steps {
                        stageStart()
                        gitPrep()

                        sh script: "./scripts/clang-format-check.sh"
                        sh script: "./scripts/clang-tidy-check.sh"
                    }
                }
            }
        }
    }
    post {
        failure {
            script {
                def automatedPr = new AutomatedPR(context: context)
                automatedPr.alertSlack()
            }
        }

        always {
            // Common post-run function from ci-jenkins shared libs.
            // Used to e.g. notify slack.
            script {
                context.slackNotify()
                context.postCommon()
            }
        }
    }
}

/**
 * Wrapper for running a make target
 * @param args
 *      args.target: make target; defaults to empty string
 * @return
 */
def runMake(Map args=[:]) {
    def context = new Context(context: this)
    def logger = context.getLogger()

    context.builder.make(
            workDir: args.workDir,
            target: args.target,
            gtestOut: args.gtestOut,
            makej: 4)
}

def createPrDescription(Map args=[:]) {
    assert args.context
    assert args.context.isPrPush()

    def pr = new PullRequest(context: args.context)
    pr.update()

    writeFile(
        file: "pr_description.yaml",
        text: """
            |---
            |commit:
            |  sha: ${pr.data.head.sha}
            |  message: '${pr.data.title}'
            |  range: ${pr.data.base.sha}..${pr.data.head.sha}
            |pr:
            |  number: ${pr.getNumber()}
            |  source_branch: ${pr.data.head.ref}
            |  sha: ${pr.data.head.sha}
            |  source_slug: ${pr.org}/${pr.repo}
            |target:
            |  branch: ${pr.data.base.ref}
            |  slug: ${pr.org}/${pr.repo}
            |test_result: 0
            |tag:
            |""".stripMargin()
    )
}

