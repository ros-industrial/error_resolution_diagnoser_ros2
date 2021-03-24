pipeline{
  agent { label 'ros2' }
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
           sh cognicept-runtest.sh
           '''
      }
    }
    stage('--cd--'){
      steps{
        echo 'Performing CD'
        sh '''
           sh cd.sh
           '''
      }
    }
    stage ("--Extract test results--") {
    steps {
      echo 'extracting test results - 2'
        sh '''
           sh ros2-test.sh
           '''
    }
    }
  }
    post {
  always {
     // junit 'coverage.xml'
      step([$class: 'CoberturaPublisher', autoUpdateHealth: false, autoUpdateStability: false, coberturaReportFile: '**/coverage.xml', failUnhealthy: false, failUnstable: false, maxNumberOfBuilds: 0, onlyStable: false, sourceEncoding: 'ASCII', zoomCoverageChart: false])
  }
}

}
