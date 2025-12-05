import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/preface">
            Get Started - Read the Preface
          </Link>
        </div>
      </div>
    </header>
  );
}

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

const FeatureList = [
  {
    title: 'Comprehensive Curriculum',
    description: (
      <>
        27 chapters covering Physical AI, ROS 2, Gazebo, Isaac Sim, VLA robotics,
        and hardware lab setup. From foundational concepts to advanced capstone projects.
      </>
    ),
  },
  {
    title: 'Hands-On Learning',
    description: (
      <>
        Practical tutorials, runnable code examples, and lab exercises using
        industry-standard tools: ROS 2, NVIDIA Isaac Sim, Whisper, and LLM integration.
      </>
    ),
  },
  {
    title: 'AI-Native Creation',
    description: (
      <>
        Created using Claude Code and Spec-Kit Plus methodology, demonstrating
        modern AI-native development workflows for educational content.
      </>
    ),
  },
];

function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}

function CourseStructure() {
  return (
    <section className={styles.courseStructure}>
      <div className="container">
        <h2 className="text--center margin-bottom--lg">Course Structure</h2>
        <div className="row">
          <div className="col col--6">
            <h3>Core Topics</h3>
            <ul>
              <li><strong>Part I</strong>: Foundations of Physical AI</li>
              <li><strong>Part II</strong>: ROS 2 - The Robotic Nervous System</li>
              <li><strong>Part III</strong>: Gazebo & Unity Simulation</li>
              <li><strong>Part IV</strong>: NVIDIA Isaac Sim</li>
            </ul>
          </div>
          <div className="col col--6">
            <h3>Advanced Topics</h3>
            <ul>
              <li><strong>Part V</strong>: Vision-Language-Action Robotics</li>
              <li><strong>Part VI</strong>: Hardware Lab Infrastructure</li>
              <li><strong>Part VII</strong>: Capstone Project</li>
              <li><strong>Part VIII</strong>: Appendices & References</li>
            </ul>
          </div>
        </div>
        <div className="text--center margin-top--lg">
          <Link
            className="button button--primary button--lg"
            to="/physical-ai/01-introduction">
            Start with Chapter 1
          </Link>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title}`}
      description="Comprehensive textbook for teaching Physical AI and Humanoid Robotics">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <CourseStructure />
      </main>
    </Layout>
  );
}
