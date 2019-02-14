/*
    There is no need to edit this file.
    All names are dynamically generated.
*/

// def getRepoURL() {
//   sh "git config --get remote.origin.url > .git/remote-url"
//   return readFile(".git/remote-url").trim()
// }

// def getCommitSha() {
//   sh "git rev-parse HEAD > .git/current-commit"
//   return readFile(".git/current-commit").trim()
// }

void setBuildStatus(String message, String context, String state) {
    withCredentials([string(credentialsId: '227fd6ff-4a49-40da-98d4-f6da7987f068', variable: 'TOKEN')]) {
      // 'set -x' for debugging. Don't worry the access token won't be actually logged
      // Also, the sh command actually executed is not properly logged, it will be further escaped when written to the log
        sh """
            set -x
            curl \"https://api.github.com/repos/org/repo/statuses/$GIT_COMMIT?access_token=$TOKEN\" \
                -H \"Content-Type: application/json\" \
                -X POST \
                -d \"{\\\"description\\\": \\\"$message\\\", \\\"state\\\": \\\"$state\\\", \\\"context\\\": \\\"$context\\\", \\\"target_url\\\": \\\"$BUILD_URL\\\"}\"
        """
    }
}


pipeline {
    agent any
    stages {
        stage('Compilation') {
            steps {
                sh "docker build -t ${env.JOB_NAME}:${env.GIT_COMMIT} .".toLowerCase().replace("%2f", "/")
            }
        }
        //stage('Unit Tests') {
        //    steps {
        //        sh "docker run ${env.JOB_NAME}:${env.GIT_COMMIT} /ros_entrypoint.sh tests".toLowerCase().replace("%2f", "/")
        //    }
        //}
    }
    post {
        success {
            setBuildStatus("Build complete", "compile", "success")
        }
        failure {
            setBuildStatus("Failed", "pl-compile", "failure")
        }
    }
}
