pipeline{
  agent { label 'ubuntu' }
  stages{
    stage('--test--'){
      steps{
        echo 'conducting tests'
      }
    }
    stage('--build--'){
      steps{
        echo 'building tests'
        sh '''
           ls
           sh cognicept-runtest.sh
           '''
      }
    }
    stage ("--Extract test results--") {
    steps {
      echo 'extracting test results - 2'
    }
    }
  }

}
