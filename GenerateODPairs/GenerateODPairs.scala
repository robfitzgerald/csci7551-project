import scala.io.Source

object GenerateODPairs {
  def main(args: Array[String]) {
    if (args.length != 2) {
      println("usage: <n> <fileName>")
      println("<n> := number of origin/destination pairs to generate")
      println("<fileName> := name of file containing transportation network graph")
      println("""  should contain a line denoting "! source-nodes" and "! destination-nodes" """)
    } else {
      val n : Integer = args(0).toInt
      val fileName : String = args(1)
      val fileLines : List[String] = Source.fromFile(fileName).getLines.toList
      val sourceNodes = fileLines.filter(_.contains("! source-nodes ")).head.split(" ").drop(2)
      val destinationNodes = fileLines.filter(_.contains("! destination-nodes")).head.split(" ").drop(2)
      val numNodes = sourceNodes.length + destinationNodes.length
      (1 to n).map(num=>{
        s"OD ${num} ${sourceNodes(num % sourceNodes.length)} ${destinationNodes(num % destinationNodes.length)}"
        }).foreach(println)
    }
  }
}