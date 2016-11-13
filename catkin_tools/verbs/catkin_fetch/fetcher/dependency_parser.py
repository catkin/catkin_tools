import os
import logging
from os import path
from xml.dom import minidom

logging.basicConfig()


class Parser(object):
    """Parses dependencies of a package."""

    XML_FILE_NAME = "package.xml"
    TAGS = ["buildtool_depend", "build_depend"]
    URL_TAGS = ["git_url"]

    def __init__(self, download_mask, pkg_name, log_level="INFO"):
        super(Parser, self).__init__()
        self.__download_mask = download_mask
        self.pkg_name = pkg_name
        self.log = logging.getLogger(pkg_name)
        self.log.setLevel(logging.getLevelName(log_level))

    def get_dependencies(self, package_folder):
        path_to_xml = Parser.__get_package_xml_path(package_folder)
        if not path_to_xml:
            self.log.critical(" 'package.xml' not found for package [%s].",
                              self.pkg_name)
            return None
        return self.__get_all_dependencies(path_to_xml)

    def __get_all_dependencies(self, path_to_xml):
        xmldoc = minidom.parse(path_to_xml)
        all_deps = []
        for tag in Parser.TAGS:
            all_deps += Parser.__node_to_list(xmldoc, tag)
        self.log.info(" Found %s dependencies: %s", len(all_deps), all_deps)
        deps_with_urls = self.__init_dep_dict(all_deps)
        return Parser.__specify_explicit_urls(xmldoc, deps_with_urls)

    @staticmethod
    def __specify_explicit_urls(xmldoc, intial_dep_dict):
        dep_dict = dict(intial_dep_dict)
        for url_tag in Parser.URL_TAGS:
            urls_node = xmldoc.getElementsByTagName(url_tag)
            for item in urls_node:
                target = item.attributes['target'].value
                url = item.attributes['url'].value
                dep_dict[target] = url
        return dep_dict

    def __init_dep_dict(self, all_deps_list):
        dep_dict = {}
        for dependency in all_deps_list:
            dep_dict[dependency] = self.__download_mask.format(
                                            package=dependency)
        return dep_dict

    @staticmethod
    def __node_to_list(xmldoc, xml_node_name):
        node = xmldoc.getElementsByTagName(xml_node_name)
        return [str(s.childNodes[0].nodeValue) for s in node]

    @staticmethod
    def __get_package_xml_path(folder):
      for file in os.listdir(folder):
        if file == Parser.XML_FILE_NAME:
            full_path = path.join(folder, file)
            return full_path
      return None
