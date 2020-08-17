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
           ls
           sh cognicept-runtest.sh
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
      step([$class: 'CoberturaPublisher', autoUpdateHealth: false, autoUpdateStability: false, coberturaReportFile: '**/ros2.xml', failUnhealthy: false, failUnstable: false, maxNumberOfBuilds: 0, onlyStable: false, sourceEncoding: 'ASCII', zoomCoverageChart: false])
  }
}

}
