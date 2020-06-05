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

      }
    }
    stage ("--Extract test results--") {
    steps {
      echo 'extracting test results'
    }
    }
  }

}
