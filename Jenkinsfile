#!groovy

// Use 'ci-jenkins@someref' to pull shared lib from a different branch/tag than the default.
// Default is configured in Jenkins and should be from "stable" tag.
@Library("ci-jenkins") import com.swiftnav.ci.*

String dockerFile = "Dockerfile"
String dockerMountArgs = "-v /mnt/efs/refrepo:/mnt/efs/refrepo"

def context = new Context(context: this)
context.setRepo("piksi_firmware_private")

def builder = context.getBuilder()
def logger = context.getLogger()
def hitl = new SwiftHitl(context: context)
hitl.update()

def macToolchain = "https://github.com/swift-nav/swift-toolchains/releases/download/pfwp_mac_toolchain/gcc-arm-none-eabi-6-2017-q2-update-mac.tar.bz2"

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
    }

    stages {
        stage('Build') {
            parallel {
                stage('Build Firmware') {
                    agent {
                      label 'macos'
/*
                        dockerfile {
                            filename dockerFile
                            args dockerMountArgs
                        }
*/
                    }
                    environment {
                        PIKSI_HW = "v3"
                        // These tokens are in plaintext so Swift developers can make PR's from their
                        // personal forks. We may want to revisit this at some point.
                        GITHUB_COMMENT_TOKEN = "29ff16b8acbf635fad0c702d3189c04a997b9719"
                        HITL_API_GITHUB_TOKEN = "d70b53ef8e8e0966818c473c56c02bf45b17290b"
                        PRODUCT_VERSION = "v3"
                        SLACK_CHANNEL = "github"
                        PATH = "$PATH:$HOME/toolchain/bin:/usr/local/bin"
                    }
                    steps {
                        stageStart()
                        gitPrep()

                        script {
                            sh("""#!/bin/bash -ex
                            | mkdir \$HOME/toolchain
                            | curl -sSL -o- ${macToolchain} | tar -xJvf - --strip-components=1 -C \$HOME/toolchain
                            | brew install cmake
                            """.stripMargin())
                            runMake(target: "PIKSI_REV=prod all")
                            runMake(target: "PIKSI_REV=base all")
                        }
                    }
                    post {
                        success {
                            script {
                              sh("echo post firmware build 3")
                          /*
                                createPrDescription(context: context)
                                context.archivePatterns(patterns: [
                                    "pr_description.yaml",
                                    "requirements.yaml",
                                    "build_v3_prod/piksi_firmware_v3_prod*.elf",
                                    "build_v3_base/piksi_firmware_v3_base*.elf"])
                                if (context.isPrPush()) {
                                    hitl.triggerForPr() // this generates metrics.yaml
                                }
                                hitl.addComments()
                          */
                            }
                        }
                        always {
                          script {
                            sh("hostname; whoami")
                            sh("reset_macos_build_node")
                          }
                        }
                    }
                }
/*
                stage('Tests & Mesta') {
                    agent {
                        dockerfile {
                            filename dockerFile
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
                            runMake(target: "run_tests")
                            runMake(target: "mesta", workDir: "mesta")
                            sh script: "./mesta/mesta"
                        }
                    }
                }
                stage('Formatting & Lint') {
                    agent {
                        dockerfile {
                            filename dockerFile
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
*/
            }
        }
    }
    post {
        always {
            // Common post-run function from ci-jenkins shared libs.
            // Used to e.g. notify slack.
            script {
              sh("echo post everything")
/*
                context.slackNotify()
                context.postCommon()
*/
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
            |  message: ${pr.data.title}
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

